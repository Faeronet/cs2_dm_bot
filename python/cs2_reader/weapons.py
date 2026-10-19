"""Имена оружия CS2 по m_iItemDefinitionIndex и режим огня (зажим / тап)."""

from __future__ import annotations

# Основные id из items_game / cs2-dumper (короткие имена для бота и логов).
WEAPON_NAMES: dict[int, str] = {
    1: "deagle",
    2: "elite",
    3: "fiveseven",
    4: "glock",
    7: "ak47",
    8: "aug",
    9: "awp",
    10: "famas",
    11: "g3sg1",
    13: "galilar",
    14: "m249",
    16: "m4a1",
    17: "mac10",
    19: "p90",
    23: "mp5sd",
    24: "ump45",
    25: "xm1014",
    26: "bizon",
    27: "mag7",
    28: "negev",
    29: "sawedoff",
    30: "tec9",
    31: "taser",
    32: "hkp2000",
    33: "mp7",
    34: "mp9",
    35: "nova",
    36: "p250",
    38: "scar20",
    39: "sg556",
    40: "ssg08",
    42: "knife",
    43: "flashbang",
    44: "hegrenade",
    45: "smokegrenade",
    46: "molotov",
    47: "decoy",
    48: "incgrenade",
    49: "c4",
    57: "healthshot",
    59: "knife_t",
    60: "m4a1_silencer",
    61: "usp_silencer",
    63: "cz75a",
    64: "revolver",
}

# Полный автомат: ЛКМ можно держать, игра сама стреляет циклом.
# FAMAS в CS2 умеет и очередь, и автомат — считаем зажимом.
# CZ75-Auto — единственный полностью автоматический пистолет.
# XM1014 — автоматический дробовик; G3SG1/SCAR-20 — автоматические винтовки.
WEAPON_AUTO: frozenset[int] = frozenset(
    {
        7,  # ak47
        8,  # aug
        10,  # famas
        11,  # g3sg1
        13,  # galilar
        14,  # m249
        16,  # m4a1 (M4A4)
        17,  # mac10
        19,  # p90
        23,  # mp5sd
        24,  # ump45
        25,  # xm1014
        26,  # bizon
        28,  # negev
        33,  # mp7
        34,  # mp9
        38,  # scar20
        39,  # sg556
        60,  # m4a1_silencer
        63,  # cz75a
    }
)

# RPM для цикла автомата (примерные значения CS2). Тап-оружие сюда не входит.
WEAPON_RPM: dict[int, float] = {
    7: 600.0,
    8: 600.0,
    10: 666.67,
    11: 240.0,
    13: 666.67,
    14: 750.0,
    16: 666.67,
    17: 800.0,
    19: 857.14,
    23: 750.0,
    24: 666.67,
    25: 240.0,
    26: 750.0,
    28: 800.0,
    33: 750.0,
    34: 857.14,
    38: 240.0,
    39: 545.45,
    60: 600.0,
    63: 600.0,
}

WEAPON_CLIP: dict[int, int] = {
    1: 7,
    2: 30,
    3: 20,
    4: 20,
    7: 30,
    8: 30,
    9: 5,
    10: 25,
    11: 20,
    13: 35,
    14: 100,
    16: 30,
    17: 30,
    19: 50,
    23: 30,
    24: 25,
    25: 7,
    26: 64,
    27: 5,
    28: 150,
    29: 7,
    30: 18,
    32: 13,
    33: 30,
    34: 30,
    35: 8,
    36: 13,
    38: 20,
    39: 30,
    40: 10,
    60: 20,
    61: 12,
    63: 12,
    64: 8,
}

_NOT_GUN: frozenset[int] = frozenset({31, 42, 43, 44, 45, 46, 47, 48, 49, 57, 59})

# Слот 2: пистолеты (нож = 3, основное = 1).
WEAPON_PISTOL: frozenset[int] = frozenset({1, 2, 3, 4, 30, 32, 36, 61, 63, 64})

# Снайперки без нормального прицела от бедра (AUG/SG553 сюда не входят — у них есть хипфайр).
WEAPON_NO_HIP_SIGHT: frozenset[int] = frozenset({9, 11, 38, 40})  # awp, g3sg1, scar20, ssg08
WEAPON_HEALTHSHOT = 57

_NAME_ALIASES: dict[str, str] = {
    "m4a4": "m4a1",
    "m4": "m4a1",
    "m4a1s": "m4a1_silencer",
    "m4a1-s": "m4a1_silencer",
    "usp": "usp_silencer",
    "usps": "usp_silencer",
    "usp-s": "usp_silencer",
    "p2000": "hkp2000",
    "galil": "galilar",
    "sg553": "sg556",
    "scout": "ssg08",
    "mac-10": "mac10",
    "mac_10": "mac10",
    "mp5": "mp5sd",
    "mp5-sd": "mp5sd",
    "mp5_sd": "mp5sd",
    "pp-bizon": "bizon",
    "pp_bizon": "bizon",
    "cz": "cz75a",
    "cz75": "cz75a",
    "r8": "revolver",
    "zeus": "taser",
    "auto": "ak47",
}

_IDS_BY_NAME: dict[str, int] = {name: item_id for item_id, name in WEAPON_NAMES.items()}
for _alias, _canon in _NAME_ALIASES.items():
    if _canon in _IDS_BY_NAME:
        _IDS_BY_NAME[_alias] = _IDS_BY_NAME[_canon]


def _norm_name(name: str) -> str:
    n = (name or "").strip().lower()
    if n.startswith("weapon_"):
        n = n[7:]
    return _NAME_ALIASES.get(n, n)


def weapon_name(item_id: int) -> str:
    if item_id in WEAPON_NAMES:
        return WEAPON_NAMES[item_id]
    if 500 <= item_id <= 525:
        return "knife"
    return f"weapon_{item_id}" if item_id else ""


def resolve_weapon(spec: int | str | None) -> tuple[int, str]:
    """id и короткое имя по индексу или строке ('ak47', 'weapon_awp', 7)."""
    if spec is None or spec == "":
        return 0, ""
    if isinstance(spec, int) or (isinstance(spec, str) and spec.strip().lstrip("-").isdigit()):
        item_id = int(spec)
        return item_id, weapon_name(item_id)
    n = _norm_name(str(spec))
    if n in _IDS_BY_NAME:
        item_id = _IDS_BY_NAME[n]
        return item_id, weapon_name(item_id)
    return 0, n


def is_gun(item_id: int, name: str = "") -> bool:
    if item_id in _NOT_GUN or 500 <= item_id <= 525:
        return False
    if item_id > 0:
        return True
    n = _norm_name(name)
    if not n:
        return False
    item_id = _IDS_BY_NAME.get(n, 0)
    if item_id in _NOT_GUN or 500 <= item_id <= 525:
        return False
    return bool(n) and n not in {"knife", "knife_t", "c4", "taser"} and "grenade" not in n


def weapon_is_pistol(item_id: int = 0, name: str = "") -> bool:
    if item_id in WEAPON_PISTOL:
        return True
    n = _norm_name(name)
    nid = _IDS_BY_NAME.get(n, 0)
    return nid in WEAPON_PISTOL


def weapon_needs_scope(item_id: int = 0, name: str = "") -> bool:
    """Нет прицела вне скоупа: AWP / SSG08 / автоснайперы."""
    if item_id in WEAPON_NO_HIP_SIGHT:
        return True
    n = _norm_name(name)
    nid = _IDS_BY_NAME.get(n, 0)
    return nid in WEAPON_NO_HIP_SIGHT


def is_healthshot(item_id: int = 0, name: str = "") -> bool:
    if item_id == WEAPON_HEALTHSHOT:
        return True
    return _norm_name(name) in {"healthshot", "medishot", "health_shot"}


def weapon_fire_mode(item_id: int = 0, name: str = "") -> str:
    """'auto' — можно зажимать, 'tap' — только одиночные, 'none' — не стреляем."""
    if item_id <= 0 and name:
        item_id, _ = resolve_weapon(name)
    if not is_gun(item_id, name):
        return "none"
    if item_id in WEAPON_AUTO:
        return "auto"
    n = _norm_name(name)
    nid = _IDS_BY_NAME.get(n, 0)
    if nid in WEAPON_AUTO:
        return "auto"
    return "tap"


def weapon_is_auto(item_id: int = 0, name: str = "") -> bool:
    return weapon_fire_mode(item_id, name) == "auto"


def weapon_rpm(item_id: int = 0, name: str = "") -> float:
    if item_id <= 0 and name:
        item_id, _ = resolve_weapon(name)
    if item_id in WEAPON_RPM:
        return WEAPON_RPM[item_id]
    n = _norm_name(name)
    nid = _IDS_BY_NAME.get(n, 0)
    if nid in WEAPON_RPM:
        return WEAPON_RPM[nid]
    return 600.0 if weapon_is_auto(item_id, name) else 0.0


def weapon_cycle_s(item_id: int = 0, name: str = "") -> float:
    rpm = weapon_rpm(item_id, name)
    return 60.0 / max(1.0, rpm)


def default_clip(item_id: int = 0, name: str = "") -> int:
    if item_id <= 0 and name:
        item_id, _ = resolve_weapon(name)
    if item_id in WEAPON_CLIP:
        return WEAPON_CLIP[item_id]
    n = _norm_name(name)
    nid = _IDS_BY_NAME.get(n, 0)
    if nid in WEAPON_CLIP:
        return WEAPON_CLIP[nid]
    return 30 if is_gun(item_id, name) else 0
