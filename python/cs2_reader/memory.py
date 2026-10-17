"""Memory backends for Linux native CS2, Windows, and minidump files."""

from __future__ import annotations

import ctypes
import os
import struct
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path

from cs2_reader.platform import Platform, client_module_name, detect_platform, engine_module_name, find_cs2_pid


class MemoryBackend(ABC):
    @abstractmethod
    def is_valid(self) -> bool: ...

    @abstractmethod
    def read(self, address: int, size: int) -> bytes | None: ...

    @abstractmethod
    def module_base(self, name: str) -> int | None: ...

    def read_u8(self, address: int) -> int | None:
        data = self.read(address, 1)
        return data[0] if data else None

    def read_u32(self, address: int) -> int | None:
        data = self.read(address, 4)
        return struct.unpack("<I", data)[0] if data else None

    def read_u16(self, address: int) -> int | None:
        data = self.read(address, 2)
        return struct.unpack("<H", data)[0] if data else None

    def read_i32(self, address: int) -> int | None:
        data = self.read(address, 4)
        return struct.unpack("<i", data)[0] if data else None

    def read_u64(self, address: int) -> int | None:
        data = self.read(address, 8)
        return struct.unpack("<Q", data)[0] if data else None

    def read_f32(self, address: int) -> float | None:
        data = self.read(address, 4)
        return struct.unpack("<f", data)[0] if data else None

    def read_vec3(self, address: int) -> tuple[float, float, float] | None:
        data = self.read(address, 12)
        return struct.unpack("<fff", data) if data else None

    def read_string(self, address: int, max_len: int = 128) -> str:
        if not address:
            return ""
        data = self.read(address, max_len)
        if not data:
            return ""
        null = data.find(b"\x00")
        if null >= 0:
            data = data[:null]
        return data.decode("utf-8", errors="replace")

    def read_cstring_ptr(self, ptr_address: int, max_len: int = 128) -> str:
        ptr = self.read_u64(ptr_address)
        if not ptr:
            return ""
        return self.read_string(ptr, max_len)


@dataclass
class ModuleInfo:
    name: str
    base: int
    size: int


def _parse_proc_maps(pid: int) -> dict[str, ModuleInfo]:
    """Parse /proc/PID/maps; handles paths with spaces."""
    modules: dict[str, ModuleInfo] = {}
    maps_path = Path(f"/proc/{pid}/maps")
    if not maps_path.exists():
        return modules

    for line in maps_path.read_text(encoding="utf-8", errors="replace").splitlines():
        parts = line.split(None, 5)
        if len(parts) < 6:
            continue
        addr_range, _perms, _offset, _dev, _inode, path = parts
        path = path.strip()
        basename = Path(path).name.lower()
        if not basename.endswith(".so") and not basename.endswith(".dll"):
            continue
        start = int(addr_range.split("-")[0], 16)
        end = int(addr_range.split("-")[1], 16)
        key = basename
        if key not in modules:
            modules[key] = ModuleInfo(name=key, base=start, size=end - start)
    return modules


class ProcMemoryBackend(MemoryBackend):
    """Read live native Linux CS2 via /proc/PID/mem."""

    def __init__(self, pid: int, platform: Platform | None = None):
        self.pid = pid
        self.platform = platform or detect_platform()
        self._mem_fd: int | None = None
        self._modules = _parse_proc_maps(pid)

    @classmethod
    def from_process_name(cls, name: str = "cs2") -> "ProcMemoryBackend | None":
        pid = find_cs2_pid()
        return cls(pid) if pid else None

    def is_valid(self) -> bool:
        return Path(f"/proc/{self.pid}").exists()

    def _open_mem(self) -> int | None:
        if self._mem_fd is not None:
            return self._mem_fd
        try:
            self._mem_fd = os.open(f"/proc/{self.pid}/mem", os.O_RDONLY)
            return self._mem_fd
        except OSError:
            return None

    def read(self, address: int, size: int) -> bytes | None:
        fd = self._open_mem()
        if fd is None or not address:
            return None
        # Валидный пользовательский диапазон x86-64 Linux: 0x1000..0x7FFFFFFFFFFF
        if address < 0x1000 or address > 0x7FFFFFFFFFFF:
            return None
        try:
            return os.pread(fd, size, address)
        except (OSError, OverflowError, ValueError):
            return None

    def write(self, address: int, data: bytes) -> bool:
        if not address:
            return False
        try:
            fd = os.open(f"/proc/{self.pid}/mem", os.O_RDWR)
        except OSError:
            return False
        try:
            os.pwrite(fd, data, address)
            return True
        except OSError:
            return False
        finally:
            os.close(fd)

    def write_f32(self, address: int, value: float) -> bool:
        return self.write(address, struct.pack("<f", value))

    def module_base(self, name: str) -> int | None:
        key = name.lower()
        mod = self._modules.get(key)
        if mod:
            return mod.base
        # Partial match for wine paths
        for mod_name, info in self._modules.items():
            if mod_name == key or mod_name.endswith("/" + key):
                return info.base
        return None

    def list_modules(self) -> dict[str, ModuleInfo]:
        return dict(self._modules)

    def close(self) -> None:
        if self._mem_fd is not None:
            os.close(self._mem_fd)
            self._mem_fd = None


class MinidumpBackend(MemoryBackend):
    """Parse Windows minidump for offline analysis."""

    def __init__(self, dump_path: Path):
        self.dump_path = dump_path
        self._data = dump_path.read_bytes()
        self._modules: dict[str, int] = {}
        self._regions: list[tuple[int, int, int]] = []
        self._parse()

    def _parse(self) -> None:
        if len(self._data) < 32:
            return
        signature = struct.unpack_from("<I", self._data, 0)[0]
        if signature != 0x504D444D:
            raise ValueError("Not a valid minidump file")

        _version, stream_count, directory_rva, *_rest = struct.unpack_from("<IIIIIII", self._data, 0)

        for i in range(stream_count):
            off = directory_rva + i * 12
            stream_type, _data_size, data_rva = struct.unpack_from("<III", self._data, off)
            if stream_type == 3:
                count = struct.unpack_from("<I", self._data, data_rva)[0]
                for j in range(count):
                    mod_off = data_rva + 4 + j * 108
                    base, _size, _cs, _ts, name_rva = struct.unpack_from("<QIIII", self._data, mod_off)
                    name_len = struct.unpack_from("<I", self._data, name_rva)[0]
                    name = self._data[name_rva + 4 : name_rva + 4 + name_len].decode("utf-16-le", errors="replace")
                    basename = Path(name).name.lower()
                    self._modules[basename] = base
            elif stream_type == 5:
                count, base_rva = struct.unpack_from("<QQ", self._data, data_rva)
                off = data_rva + 16
                file_off = base_rva
                for _ in range(count):
                    start, size = struct.unpack_from("<QQ", self._data, off)
                    off += 16
                    self._regions.append((start, size, file_off))
                    file_off += size

    def is_valid(self) -> bool:
        return bool(self._regions)

    def read(self, address: int, size: int) -> bytes | None:
        for start, region_size, file_off in self._regions:
            if start <= address < start + region_size:
                rel = address - start
                if rel + size > region_size:
                    return None
                return self._data[file_off + rel : file_off + rel + size]
        return None

    def module_base(self, name: str) -> int | None:
        return self._modules.get(name.lower())


class WindowsProcessBackend(MemoryBackend):
    def __init__(self, pid: int):
        self.pid = pid
        self._handle = None
        self._modules: dict[str, int] = {}
        self._open()
        self._load_modules()

    @classmethod
    def from_process_name(cls, name: str = "cs2.exe") -> "WindowsProcessBackend | None":
        pid = find_cs2_pid()
        return cls(pid) if pid else None

    def _open(self) -> None:
        kernel32 = ctypes.windll.kernel32
        access = 0x0010 | 0x0400
        self._handle = kernel32.OpenProcess(access, False, self.pid)

    def _load_modules(self) -> None:
        import subprocess

        result = subprocess.run(
            [
                "powershell",
                "-Command",
                f"(Get-Process -Id {self.pid}).Modules | Select-Object ModuleName,BaseAddress | ConvertTo-Csv -NoTypeInformation",
            ],
            capture_output=True,
            text=True,
        )
        for line in result.stdout.splitlines()[1:]:
            parts = line.replace('"', "").split(",")
            if len(parts) >= 2:
                try:
                    base = int(parts[1], 16) if parts[1].startswith("0x") else int(parts[1])
                    self._modules[parts[0].lower()] = base
                except ValueError:
                    continue

    def is_valid(self) -> bool:
        return bool(self._handle)

    def read(self, address: int, size: int) -> bytes | None:
        if not self._handle:
            return None
        buf = ctypes.create_string_buffer(size)
        read = ctypes.c_size_t(0)
        ok = ctypes.windll.kernel32.ReadProcessMemory(
            self._handle, ctypes.c_void_p(address), buf, size, ctypes.byref(read)
        )
        if not ok or read.value != size:
            return None
        return buf.raw

    def module_base(self, name: str) -> int | None:
        return self._modules.get(name.lower())


def create_live_backend() -> MemoryBackend | None:
    if detect_platform() == Platform.WINDOWS:
        return WindowsProcessBackend.from_process_name("cs2.exe")
    return ProcMemoryBackend.from_process_name("cs2")
