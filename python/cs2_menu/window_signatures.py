"""Сигнатуры окон CS2 по памяти (офсет Panorama) + наведение/закрытие кнопок.

Идея (как просил пользователь): «из офсета составить общую сигнатуру окон»
(ошибки/уведомления/Prime/…), искать кнопку OK / Close / «больше не показывать»,
наводить на неё курсор и закрывать.

Что реально дают офсеты:
  * `libpanorama.so + MENU_CONTEXT` → указатель контекста Panorama. Хэш 512 байт
    этого контекста (UIState.ui_fingerprint) — СТАБИЛЬНАЯ сигнатура ТЕКУЩЕГО окна/
    экрана. По ней надёжно отличаем «чистое меню» от конкретного попапа.

Чего офсеты НЕ дают:
  * координат кнопок внутри окна — дерево панелей Panorama (с прямоугольниками/
    текстом кнопок) в игровой схеме не выставлено, OCR в системе нет. Поэтому
    позиции кнопок (OK / Close / «не показывать») храним ПО СИГНАТУРЕ как доли
    КОНТЕНТА окна (устойчивы к разрешению) — калибруются один раз инструментом learn.

Как работает:
  1) читаем ui_fingerprint из памяти → классифицируем окно (реестр сигнатур);
  2) берём кнопки этой сигнатуры (по приоритету: dont_show → close → ok);
  3) НАВОДИМ курсор на кнопку, проверяем подсветку (Panorama светлеет при hover),
     затем кликаем; повторяем, пока окно не исчезнет (по памяти/яркости);
  4) неизвестное окно → откат на яркостный детектор (modal_vision).

Реестр хранится в data/window_signatures.json и дополняется на лету (learn).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

from PIL import Image, ImageStat

from cs2_menu.modal_vision import clear_modals, detect_modal, grab_window

ROOT = Path(__file__).resolve().parents[2]
REGISTRY_PATH = ROOT / "data" / "window_signatures.json"

# Насколько ярче должна стать область кнопки при наведении, чтобы счесть hover «попал».
# Реальная кнопка подсвечивается заметно (CLOSE Prime ≈+28, OK диалога ≈+16); яркий
# контент модалки даёт лишь ≈+6 на наведении — поэтому порог 12 отсекает ложные.
HOVER_LUMA_DELTA = 12.0

# Порядок приоритета кнопок при закрытии окна.
_BUTTON_PRIORITY = {"dont_show": 0, "close": 1, "ok": 2, "yes": 3, "accept": 4}


@dataclass
class Button:
    label: str          # ok / close / dont_show / yes / accept
    fx: float           # доля КОНТЕНТА окна по X
    fy: float           # доля КОНТЕНТА окна по Y

    def to_dict(self) -> dict:
        return {"label": self.label, "fx": round(self.fx, 4), "fy": round(self.fy, 4)}


@dataclass
class WindowSig:
    id: str
    description: str = ""
    is_clean: bool = False                      # «чистое» состояние (окна нет)
    fingerprints: set[str] = field(default_factory=set)
    buttons: list[Button] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "description": self.description,
            "is_clean": self.is_clean,
            "fingerprints": sorted(self.fingerprints),
            "buttons": [b.to_dict() for b in self.buttons],
        }

    def ordered_buttons(self) -> list[Button]:
        return sorted(self.buttons, key=lambda b: _BUTTON_PRIORITY.get(b.label, 9))


# ───────────────────────── реестр (JSON) ─────────────────────────

def _default_registry() -> dict[str, WindowSig]:
    """Стартовый набор: позиции кнопок — доли контента (из modal_vision/калибровки)."""
    return {
        "clean_menu": WindowSig(
            id="clean_menu", description="Чистое меню (окна нет)", is_clean=True,
        ),
        "error": WindowSig(
            id="error", description="Ошибка/подтверждение (один OK)",
            # OK центрированной панели (позиция зависит от ширины диалога): несколько
            # típичных центров + «Disconnected»/update-ошибки.
            buttons=[Button("ok", 0.612, 0.552),   # Disconnected / Unable to connect
                     Button("ok", 0.666, 0.561),   # узкая панель
                     Button("ok", 0.500, 0.561)],  # центр
        ),
        "prime_upsell": WindowSig(
            id="prime_upsell", description="Prime / магазин upsell",
            buttons=[Button("close", 0.720, 0.933)],
        ),
        "notification": WindowSig(
            id="notification", description="Уведомление / toast",
            buttons=[Button("close", 0.720, 0.933)],
        ),
        # Overlay-диалог: НЕ меняет fingerprint (MENU_CONTEXT = экран под ним),
        # ловится по яркости/подсветке. Калибровано парсингом окна.
        "community_servers": WindowSig(
            id="community_servers",
            description="Диалог 'Community Servers' (never-show / OK / Cancel)",
            buttons=[Button("dont_show", 0.520, 0.583),
                     Button("ok", 0.627, 0.583),
                     Button("close", 0.664, 0.583)],
        ),
        # Полноэкранный под-экран (инспекция предмета/charm, карточка магазина):
        # яркая 3D-сцена, кнопка CLOSE в правом-нижнем углу.
        "fullscreen_sub": WindowSig(
            id="fullscreen_sub",
            description="Под-экран инспекции/магазина (CLOSE справа-внизу)",
            buttons=[Button("close", 0.756, 0.953)],
        ),
    }


# «Универсальные» точки кнопки закрытия (доли контента) — пробуем кликом напрямую у
# неизвестных окон (hover-подсветка на ярких 3D-сценах ненадёжна).
_UNIVERSAL_CLOSE: list[tuple[float, float, str]] = [
    (0.756, 0.953, "close-bottom-right"),  # CLOSE под-экрана инспекции/магазина
    (0.720, 0.933, "close-promo"),         # CLOSE промо-модалки (Prime)
]


def load_registry() -> dict[str, WindowSig]:
    reg = _default_registry()
    if REGISTRY_PATH.is_file():
        try:
            raw = json.loads(REGISTRY_PATH.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            raw = {}
        for sid, item in raw.items():
            buttons = [Button(b["label"], float(b["fx"]), float(b["fy"]))
                       for b in item.get("buttons", [])]
            sig = WindowSig(
                id=sid,
                description=item.get("description", ""),
                is_clean=bool(item.get("is_clean", False)),
                fingerprints=set(item.get("fingerprints", [])),
                buttons=buttons or (reg[sid].buttons if sid in reg else []),
            )
            # сохранить дефолтное описание/кнопки, если в файле пусто
            if sid in reg and not sig.description:
                sig.description = reg[sid].description
            reg[sid] = sig
    return reg


def save_registry(reg: dict[str, WindowSig]) -> None:
    REGISTRY_PATH.parent.mkdir(parents=True, exist_ok=True)
    data = {sid: sig.to_dict() for sid, sig in reg.items()}
    REGISTRY_PATH.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n",
                             encoding="utf-8")


def learn_fingerprint(sig_id: str, fingerprint: str, *,
                      description: str = "", buttons: list[Button] | None = None,
                      is_clean: bool = False) -> WindowSig:
    """Добавить текущий fingerprint (и, опционально, кнопки) к сигнатуре sig_id."""
    reg = load_registry()
    sig = reg.get(sig_id) or WindowSig(id=sig_id, description=description, is_clean=is_clean)
    if fingerprint:
        sig.fingerprints.add(fingerprint)
    if description:
        sig.description = description
    if is_clean:
        sig.is_clean = True
    if buttons:
        sig.buttons = buttons
    reg[sig_id] = sig
    save_registry(reg)
    return sig


# ───────────────────────── классификация ─────────────────────────

@dataclass
class WindowVerdict:
    sig: WindowSig | None            # распознанная сигнатура (или None)
    source: str                      # memory / vision / none
    fingerprint: str = ""
    modal_present: bool = False      # по яркости (vision)
    ring_mean: float = 0.0

    @property
    def window_open(self) -> bool:
        if self.sig is not None and not self.sig.is_clean:
            return True
        if self.sig is not None and self.sig.is_clean:
            return False
        return self.modal_present


def classify(fingerprint: str, *, clicker=None, env: dict | None = None,
             registry: dict[str, WindowSig] | None = None) -> WindowVerdict:
    """Определить текущее окно: сначала по памяти (fingerprint), затем по яркости."""
    reg = registry or load_registry()
    # 1) память: точное совпадение fingerprint
    for sig in reg.values():
        if fingerprint and fingerprint in sig.fingerprints:
            return WindowVerdict(sig=sig, source="memory", fingerprint=fingerprint)
    # 2) яркость (vision) — для неизвестных окон
    modal = False
    ring = 0.0
    if clicker is not None and env is not None:
        clicker._mi.focus()
        rect = clicker.content_rect()
        if rect:
            img = grab_window(env, rect)
            if img is not None:
                info = detect_modal(img)
                modal, ring = info.present, info.ring_mean
    return WindowVerdict(sig=None, source="vision" if modal else "none",
                         fingerprint=fingerprint, modal_present=modal, ring_mean=ring)


# ───────────────────────── наведение + клик ─────────────────────────

def _region_luma(img: Image.Image, px: int, py: int, half: int = 16) -> float:
    w, h = img.size
    box = (max(0, px - half), max(0, py - half),
           min(w, px + half), min(h, py + half))
    if box[2] <= box[0] or box[3] <= box[1]:
        return 0.0
    return ImageStat.Stat(img.crop(box).convert("L")).mean[0]


def hover_button(clicker, env: dict, btn: Button) -> dict:
    """Навести курсор на кнопку и проверить подсветку (без клика).

    Доли — от игрового поля. Возвращает {screen, highlighted, luma_before, luma_after}.
    """
    clicker._mi.focus()
    before = clicker.capture_client()
    if before is None:
        return {"screen": (0, 0), "luma_before": 0.0, "luma_after": 0.0, "highlighted": False}
    px, py = before.client.pixel(btn.fx, btn.fy)
    sx, sy = before.screen(btn.fx, btn.fy)
    lb = _region_luma(before.image, px, py)
    clicker.hover_screen(sx, sy)
    after = clicker.capture_client()
    if after is None:
        la = 0.0
    else:
        la = _region_luma(after.image, *after.client.pixel(btn.fx, btn.fy))
    return {
        "screen": (sx, sy),
        "luma_before": round(lb, 1),
        "luma_after": round(la, 1),
        "highlighted": (la - lb) >= HOVER_LUMA_DELTA,
    }


def press_button(clicker, env: dict, btn: Button, *, hover_first: bool = True) -> dict:
    """Навести (с проверкой подсветки) и кликнуть кнопку."""
    info = {"highlighted": None}
    if hover_first:
        info = hover_button(clicker, env, btn)
    clicker.act_client(btn.fx, btn.fy, settle=0.9)
    return info


def _still_open(clicker, env: dict, reader_state_fn, registry) -> WindowVerdict:
    fp = ""
    if reader_state_fn is not None:
        try:
            fp = reader_state_fn()
        except Exception:  # noqa: BLE001
            fp = ""
    return classify(fp, clicker=clicker, env=env, registry=registry)


def _confirmed_closed(clicker, env: dict, reader_state_fn, registry) -> bool:
    """Окно действительно закрыто, если ДВА подряд кадра (с паузой) говорят «нет окна».

    Защита от ложного успеха: яркая 3D-сцена может на один кадр «просветлеть» и
    выглядеть как закрытие — требуем устойчивости.
    """
    if _still_open(clicker, env, reader_state_fn, registry).window_open:
        return False
    time.sleep(0.4)
    return not _still_open(clicker, env, reader_state_fn, registry).window_open


def _all_candidate_buttons(registry: dict[str, WindowSig]) -> list[Button]:
    """Уникальные кнопки из всех НЕ-чистых сигнатур (для перебора у неизвестных окон)."""
    seen: set[tuple[int, int]] = set()
    out: list[Button] = []
    sigs = sorted((s for s in registry.values() if not s.is_clean),
                  key=lambda s: 0 if s.buttons else 1)
    for sig in sigs:
        for b in sig.ordered_buttons():
            key = (round(b.fx, 3), round(b.fy, 3))
            if key in seen:
                continue
            seen.add(key)
            out.append(b)
    return out


# Макс. базовая яркость области, чтобы счесть её кнопкой диалога. Реальные OK/Close
# сидят на ТЁМНОЙ панели (базовая luma ~50-65). Яркий контент (превью предметов,
# плитки карт, 3D-модель) имеет базовую luma >100 — так отсекаем ложные клики по нему.
DARK_BASELINE_MAX = 105.0

# Приоритетные точки кнопок OK/Close (доли КОНТЕНТА). Перебираем у окон, которые НЕ
# опознаются ни по памяти, ни по затемнению фона — например, «Disconnected»/«Unable to
# connect» фон не затемняет, поэтому яркостный детектор его не видит.
PROBE_BUTTONS: list[tuple[float, float, str]] = [
    (0.612, 0.552, "ok"),      # Disconnected / Unable to establish connection
    (0.666, 0.561, "ok"),      # узкая панель OK
    (0.500, 0.561, "ok"),      # центр OK
    (0.627, 0.583, "ok"),      # Community Servers OK
    (0.664, 0.583, "cancel"),  # Community Servers Cancel
    (0.720, 0.933, "close"),   # промо/уведомление CLOSE (Prime)
    (0.756, 0.953, "close"),   # под-экран инспекции/магазина CLOSE
]


def _looks_like_button(res: dict) -> bool:
    """Это кнопка диалога: подсветилась при наведении (Δ≥порог) И базовая яркость —
    тёмная (панель), а не яркий контент (плитка/превью/модель)."""
    return bool(res["highlighted"]) and res["luma_before"] <= DARK_BASELINE_MAX


def _press_if_button(clicker, env: dict, fx: float, fy: float, label: str, log) -> bool:
    """Навести на точку; если это кнопка (тёмная база + подсветка) — кликнуть и
    подтвердить закрытие по ИСЧЕЗНОВЕНИЮ кнопки (повторный hover уже не кнопка).

    Подтверждение по исчезновению кнопки не зависит от яркости фона — поэтому ловит и
    «Disconnected» (фон не темнеет), и под-экран инспекции (яркая 3D-сцена).
    """
    res = hover_button(clicker, env, Button(label, fx, fy))
    if not _looks_like_button(res):
        return False
    d = res["luma_after"] - res["luma_before"]
    log(f"[win]   кнопка {label} @({fx:.3f},{fy:.3f}) luma "
        f"{res['luma_before']}→{res['luma_after']} (Δ{d:+.0f}) → клик")
    clicker.act_client(fx, fy, settle=0.9)
    res2 = hover_button(clicker, env, Button(label, fx, fy))
    if not _looks_like_button(res2):
        log(f"[win]   закрыто — кнопка исчезла (luma {res2['luma_before']}→{res2['luma_after']})")
        return True
    log("[win]   клик не закрыл — кнопка на месте")
    return False


def _probe_close(clicker, env: dict, log) -> bool:
    """Перебор известных точек OK/Close по правилу «тёмная база + подсветка»."""
    for fx, fy, label in PROBE_BUTTONS:
        if _press_if_button(clicker, env, fx, fy, label, log):
            return True
    return False


def _read_fp(reader_state_fn) -> str:
    if reader_state_fn is None:
        return ""
    try:
        return reader_state_fn() or ""
    except Exception:  # noqa: BLE001
        return ""


@dataclass
class DetectResult:
    """Результат ДЕТЕКЦИИ окна (без кликов) — по сигнатуре."""

    present: bool
    source: str                                   # memory / vision-ring / hover-probe / none
    sig: WindowSig | None = None                  # опознанная по памяти сигнатура
    ring_mean: float = 0.0                        # яркость рамки (затемнение фона)
    probe_hit: tuple[float, float, str] | None = None  # (fx,fy,label) кнопки, найденной наведением


def _detect_window(clicker, env: dict, reader_state_fn, registry) -> DetectResult:
    """Есть ли сейчас окно/уведомление/ошибка — ТОЛЬКО детекция, без кликов и ESC.

    Надёжные и БЫСТРЫЕ признаки (один кадр, без наведения):
      1) ПАМЯТЬ — fingerprint совпал с известной НЕ-чистой сигнатурой;
      2) ЯРКОСТЬ — фон затемнён модалкой (ring < порога) — modal_vision.

    ВАЖНО: наведение (hover-подсветка) НЕ используется для детекции присутствия —
    на обычных экранах (например, «идёт поиск матча») интерактивные элементы дают
    слабую подсветку (Δ≈16) неотличимую от не-затемняющего диалога, что раньше
    приводило к ложному срабатыванию и зависанию. Поэтому присутствие окна
    определяем только по затемнению фона/памяти; наведение — лишь при ЗАКРЫТИИ.
    """
    # 1) память
    fp = _read_fp(reader_state_fn)
    for sig in registry.values():
        if fp and fp in sig.fingerprints and not sig.is_clean:
            return DetectResult(True, "memory", sig=sig)

    # 2) яркость (затемнение фона модалкой) — один кадр
    clicker._mi.focus()
    from cs2_menu.modal_vision import _client_view, _modal_on_client
    fr = _client_view(clicker)
    if fr is not None:
        _crop, info = _modal_on_client(fr)
        if info.present:
            return DetectResult(True, "vision-ring", ring_mean=info.ring_mean)
        from cs2_menu.live_ui import find_inspect_close
        close = find_inspect_close(fr.image, fr.client)
        if close is not None:
            return DetectResult(True, "inspect-close", probe_hit=(close[0], close[1], "close"),
                                ring_mean=info.ring_mean)
        return DetectResult(False, "none", ring_mean=info.ring_mean)
    return DetectResult(False, "none")


def _esc_close(clicker, env: dict, reader_state_fn, registry, log, attempts: int = 3) -> bool:
    """Закрыть окно клавишей ESC. На XWayland/mutter клавиша доходит до CS2 только
    когда окно активно, а `windowactivate --sync` срабатывает НЕ всегда (таймаут 2с),
    поэтому пробуем несколько раз: перед каждым ESC заново активируем окно и проверяем
    результат. Как только окно исчезло — успех (без наведения/кликов)."""
    mi = clicker._mi
    wid = str(mi._window_id) if getattr(mi, "_window_id", None) else ""
    for a in range(attempts):
        if wid:
            mi._xdo("windowactivate", "--sync", wid)
            time.sleep(0.25)
        try:
            mi._inp.key_tap("escape", hold_ms=60)
        except Exception as exc:  # noqa: BLE001
            log(f"[win] ESC не отправился ({exc})")
            break
        time.sleep(0.7)
        if not _detect_window(clicker, env, reader_state_fn, registry).present:
            log(f"[win] закрыто по ESC ✓ (попытка {a + 1})")
            return True
    return False


def _close_one(clicker, env: dict, det: DetectResult, reader_state_fn, registry, log) -> bool:
    """Закрыть ОДНО обнаруженное окно: ESC (несколько попыток) → наведение+клик."""
    # 1) ESC с повторами и активацией окна
    log("[win] жму ESCAPE…")
    if _esc_close(clicker, env, reader_state_fn, registry, log):
        return True

    # осмотр предмета / окно дропа: белая CLOSE, зелёную «получить» не трогаем
    if det.source == "inspect-close" and det.probe_hit is not None:
        fx, fy, label = det.probe_hit
        log(f"[win] закрываю осмотр {label} @({fx:.3f},{fy:.3f})")
        clicker.act_client(fx, fy, settle=0.8)
        if not _detect_window(clicker, env, reader_state_fn, registry).present:
            return True
    # 2) наведение+клик (один проход)
    log("[win] ESC не закрыл — навожу курсор на кнопку закрытия")
    if det.sig is not None and det.sig.buttons:
        for btn in det.sig.ordered_buttons():
            if btn.label == "dont_show":
                clicker.act_client(btn.fx, btn.fy, settle=0.5)  # «не показывать»
                continue
            if _press_if_button(clicker, env, btn.fx, btn.fy, btn.label, log):
                return True
    if _probe_close(clicker, env, log):
        return True

    # 3) затемнение осталось, кнопку не нашли → яркостный откат (один раз)
    cur = _detect_window(clicker, env, reader_state_fn, registry)
    if cur.present and cur.source == "vision-ring":
        log("[win] кнопку не нашёл — яркостный откат")
        if clear_modals(clicker, env, log=log):
            return True
    return False


def close_window(clicker, env: dict, *, reader_state_fn=None, max_rounds: int = 5,
                 log=print) -> bool:
    """Распознать окно(а) по сигнатуре и закрыть. НЕ зависает (кадр окна ~0.4с).

    Поток (как просил пользователь): на каждой итерации —
      1) ДЕТЕКЦИЯ по сигнатуре (память / затемнение фона). Нет окна → выходим (идём
         дальше по PLAY). CS2 показывает промо ПОДРЯД (Community Servers → Prime → …),
         поэтому повторяем до max_rounds, пока экран не станет чистым.
      2) Окно есть → СНАЧАЛА ESCAPE; если не закрылось — ОДИН проход наведения курсора
         на кнопку (OK / Close / «не показывать») с подтверждением по исчезновению.
      3) Окно не поддаётся — выходим (поток не блокируем, без бесконечных попыток).
    """
    reg = load_registry()
    closed_any = False
    for i in range(max_rounds):
        det = _detect_window(clicker, env, reader_state_fn, reg)
        if not det.present:
            log("[win] все окна закрыты — идём дальше" if closed_any
                else "[win] окно не обнаружено — идём дальше")
            return True
        extra = ""
        if det.sig is not None:
            extra = f" '{det.sig.id}'"
        elif det.ring_mean:
            extra = f" ring={det.ring_mean:.0f}"
        log(f"[win] окно #{i + 1} [{det.source}]{extra} — закрываю")
        if not _close_one(clicker, env, det, reader_state_fn, reg, log):
            log("[win] это окно не закрылось — выхожу (поток не блокирую)")
            return closed_any
        closed_any = True
    log(f"[win] достигнут предел ({max_rounds}) — окно всплывает снова, выхожу")
    return closed_any


def review_store_drop(clicker, log=print) -> bool:
    """Открыть магазин, прокликать предметы дропа, окна закрыть, на «получить» только навестись."""
    from cs2_menu.live_ui import find_drop_cards, find_inspect_close, find_nav, find_team_select

    clicker._mi.focus()
    fr = clicker.capture_client()
    if fr is None:
        log("[drop] окно CS2 не найдено")
        return False
    nav = find_nav(fr.image, fr.client)
    teams = find_team_select(fr.image, fr.client)
    if "cancel" in teams and "store" not in nav:
        log("[drop] открыт выбор стороны — выхожу в меню")
        clicker.click_live("cancel", settle=1.3)
    fr = clicker.capture_client()
    on_store = False
    if fr is not None:
        from cs2_menu.live_ui import find_band
        tabs = find_band(
            fr.image, fr.client, ("home", "general", "market", "tools", "armory"), 0.055, 0.11,
        )
        on_store = "general" in tabs and "market" in tabs
    if not on_store:
        clicker.click_live("store", settle=1.1)
    clicker.click_live("store_home", settle=1.0)
    fr = clicker.capture_client()
    cards = find_drop_cards(fr.image, fr.client) if fr else []
    if len(cards) != 4 and fr is not None and "store" in find_nav(fr.image, fr.client):
        from cs2_menu.live_ui import find_green
        if find_green(fr.image, fr.client) is not None:
            cards = [(0.218, 0.247), (0.321, 0.247), (0.423, 0.247), (0.525, 0.247)]
            log("[drop] карточки по кадру не собрались, беру записанные четыре")
    if len(cards) != 4:
        log("[drop] блок дропа на экране не найден")
        return False
    log(f"[drop] карточек: {len(cards)}")
    for i, (fx, fy) in enumerate(cards, 1):
        log(f"[drop] предмет {i} @({fx:.3f},{fy:.3f})")
        clicker.act_client(fx, fy, settle=1.0)
        _close_drop_view(clicker, log)
    hovered = clicker.click_live("claim", hover=True, settle=0.4)
    log("[drop] курсор на «получить», кнопка не нажата" if hovered
        else "[drop] кнопку «получить» на кадре не нашёл")
    return hovered


def _close_drop_view(clicker, log) -> None:
    """Закрыть окно осмотра предмета. Пока видна верхняя навигация — это ещё магазин, ESC не жмём."""
    from cs2_menu.live_ui import find_inspect_close, find_nav
    from cs2_menu.modal_vision import clear_modals

    fr = clicker.capture_client()
    if fr is None:
        return
    if find_nav(fr.image, fr.client):
        log("[drop]   отдельное окно не открылось")
        return
    try:
        clicker._mi.game_input.key_tap("escape", hold_ms=60)
    except Exception as exc:  # noqa: BLE001
        log(f"[drop]   ESC не отправился ({exc})")
    time.sleep(0.7)
    fr = clicker.capture_client()
    if fr is not None and find_nav(fr.image, fr.client):
        log("[drop]   окно предмета закрыто")
        return
    if fr is not None:
        close = find_inspect_close(fr.image, fr.client)
        if close is not None:
            log("[drop]   жму CLOSE осмотра")
            clicker.act_client(*close, settle=0.8)
            return
    clear_modals(clicker, getattr(clicker, "_env", {}) or {}, max_rounds=1, log=log)
