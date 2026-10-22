"""Minimal Steam VDF read/write (KeyValues text format)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class VdfNode:
    value: str | None = None
    children: dict[str, VdfNode] | None = None

    def is_block(self) -> bool:
        return self.children is not None


def _tokenize(text: str) -> list[str]:
    tokens: list[str] = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
            continue
        if c == "{":
            tokens.append("{")
            i += 1
            continue
        if c == "}":
            tokens.append("}")
            i += 1
            continue
        if c != '"':
            raise ValueError(f"unexpected char {c!r} at {i}")
        i += 1
        buf: list[str] = []
        while i < n:
            c = text[i]
            if c == "\\" and i + 1 < n:
                buf.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                i += 1
                break
            buf.append(c)
            i += 1
        tokens.append("".join(buf))
    return tokens


def parse_vdf(text: str) -> VdfNode:
    tokens = _tokenize(text)
    idx = 0

    def parse_block() -> VdfNode:
        nonlocal idx
        node = VdfNode(children={})
        assert node.children is not None
        while idx < len(tokens):
            tok = tokens[idx]
            if tok == "}":
                idx += 1
                return node
            key = tok
            idx += 1
            if idx >= len(tokens):
                raise ValueError("unexpected EOF after key")
            nxt = tokens[idx]
            if nxt == "{":
                idx += 1
                node.children[key] = parse_block()
            else:
                node.children[key] = VdfNode(value=nxt)
                idx += 1
        return node

    if not tokens:
        return VdfNode(children={})
    if tokens[0] == "{":
        idx = 1
        return parse_block()
    root_key = tokens[0]
    idx = 1
    if idx < len(tokens) and tokens[idx] == "{":
        idx += 1
        root = parse_block()
        root_key_node = VdfNode(children={root_key: root})
        return root_key_node
    raise ValueError("unsupported VDF root")


def _serialize(node: VdfNode, indent: int = 0) -> list[str]:
    lines: list[str] = []
    pad = "\t" * indent
    if node.children is None:
        return lines
    for key, child in node.children.items():
        if child.is_block():
            lines.append(f'{pad}"{key}"')
            lines.append(f"{pad}{{")
            lines.extend(_serialize(child, indent + 1))
            lines.append(f"{pad}}}")
        else:
            val = child.value or ""
            lines.append(f'{pad}"{key}"\t\t"{val}"')
    return lines


def serialize_vdf(root: VdfNode) -> str:
    if not root.children or len(root.children) != 1:
        raise ValueError("expected single-root VDF")
    key = next(iter(root.children))
    child = root.children[key]
    lines = [f'"{key}"', "{"]
    lines.extend(_serialize(child, 1))
    lines.append("}")
    return "\n".join(lines) + "\n"


def get_ci(node: VdfNode, key: str) -> VdfNode | None:
    if not node.children:
        return None
    for k, v in node.children.items():
        if k.lower() == key.lower():
            return v
    return None


def set_ci(node: VdfNode, key: str, value: str) -> None:
    if not node.children:
        node.children = {}
    for k in list(node.children):
        if k.lower() == key.lower():
            node.children[k].value = value
            return
    node.children[key] = VdfNode(value=value)
