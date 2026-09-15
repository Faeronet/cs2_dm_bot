# CS2 Memory Reader (Linux Native)

Python-драйвер для чтения памяти нативного CS2 на Linux.

## Структура

```
cs2-memory-damp/
├── python/cs2_reader/       # Драйвер чтения памяти
├── python/cs2_control/      # Ввод: клавиши (uinput) и мышь
├── python/cs2_nav/          # Навигация: карта → маршрут → контроллер, симулятор
├── python/tools/            # CLI утилиты
├── scripts/                 # Оффсеты и валидация
├── data/
│   ├── offsets.json         # Оффсеты (после dump_live)
│   └── recordings/          # Записанные координаты локаций
└── external/cs2-dumper/     # Снятие оффсетов
```

## Использование

```bash
source .venv/bin/activate

# Live чтение (CS2 должен быть запущен)
python3 python/tools/live_reader.py --once --json

# Валидация оффсетов
python3 scripts/validate_live.py

# Запись координат локации (бегаешь по карте, stop — сохранить)
python3 python/tools/record_coords.py -m de_dust2 -l lower_tunel

# Обновить оффсеты после патча CS2
bash scripts/dump_live.sh
python3 scripts/update_offsets.py
```

## Запись координат

```bash
python3 python/tools/record_coords.py -m de_dust2 -l b_tunnel
# бег по зоне, затем в консоли: stop
# → data/recordings/de_dust2/b_tunnel.json
python3 python/tools/record_coords.py --list
```

## Прицел и ввод

```bash
# Проверка позиции и прицела (глаза, pitch/yaw, точка на полу)
python3 python/tools/check_pose.py
python3 python/tools/check_pose.py --watch 5
python3 python/tools/check_pose.py --toward -1200 1340

# Куда смотрит прицел (live или дамп)
python3 python/tools/test_control.py --read-aim
python3 python/tools/test_control.py --dump game.dmp --read-aim

# Тест клавиш / мыши (резко, CS2 в фокусе)
python3 python/tools/test_control.py --key w --hold 2
python3 python/tools/test_control.py --mouse 400 0

# Тест-драйв: крестик → мышь → круг (читает прицел из памяти)
python3 python/tools/walk_test.py
python3 python/tools/walk_test.py --leg 1.5 --circle 8

```

## Навигация

Бот строит карту локации из записанных координат, выбирает самую дальнюю
достижимую точку и идёт к ней камерой + W/A/S/D. Никаких «зашитых» знаний о
локации: новая/расширенная запись подхватывается автоматически.

```bash
# Живой запуск (CS2 в фокусе). По умолчанию data/recordings/de_dust2/tunels.json
python3 python/tools/walk_nav.py --duration 30
python3 python/tools/walk_nav.py -r data/recordings/de_dust2/lower_tunel1.json --loop --png /tmp/route.png

# Карта локации как PNG-радар (проверить, что запись «сложилась»)
python3 python/tools/render_map.py -o /tmp/map.png --clearance

# Офлайн-симуляция без CS2: маршрут + физика + метрики + PNG
python3 python/tools/sim_nav.py --random-starts 8 --out /tmp/sim.png
python3 python/tools/sim_nav.py --invert-mouse --mem-yaw-offset 90 --noise 2   # стресс-тест калибровки
python3 python/tools/sim_nav.py --random-targets 4 --random-starts 4           # мишени-макеты на карте
python3 python/tools/sim_nav.py --targets="-1116,1180,-50;-1084,1176,-52"      # мишени в конкретных точках
python3 python/tools/sim_nav.py --random-targets 3 --no-fire                     # смотреть на мишени, не стрелять
python3 python/tools/sim_nav.py --random-targets 3 --hits-to-kill 0              # мишени не пропадают (предохранитель 6–8 с)
python3 python/tools/walk_nav.py --duration 60                                   # живой: стоп → флик → стрелять, пока мишень не пропадёт
python3 python/tools/walk_nav.py --duration 60 --shots 3                         # или лимит выстрелов
python3 python/tools/walk_nav.py --duration 60 --aim-z 48                        # точка прицеливания ниже (u над ногами мишени; 56 = верх груди)

# Склеить несколько записей в один файл (перекрытия на стыке убираются, radius 8u)
python3 python/tools/merge_recordings.py -l tunels \
    data/recordings/de_dust2/lower_tunel1.json data/recordings/de_dust2/uper_tunel.json
```

Как это работает (`python/cs2_nav/`):

| Модуль | Что делает |
|---|---|
| `coord_map.py` | 2.5D-сетка проходимости: точки записи → «капсулы» шириной с игрока (32u) → морфологическое закрытие (щели между полосами бега заполняются, реальные препятствия — нет) → кластеризация по Z (винтовые лестницы, ящики) → рёбра step/jump/drop → поле clearance (расстояние до стены). |
| `pathfinder.py` | Dijkstra до самой дальней достижимой точки; A* с штрафом за близость к стенам; сглаживание маршрута (string pulling) с сохранением локального clearance. |
| `route.py` | Полилиния: проекция позиции, точка «через N юнитов», уклон и поворот впереди. |
| `navigator.py` | Контроллер «как игрок»: камера плавно ведёт точку маршрута на 120–260u впереди (зависит от скорости), ноги идут за «морковкой» на 60u со steering по текущей скорости из памяти; 8 направлений WASD с гистерезисом; торможение у цели; онлайн-калибровка °/px мыши и yaw-offset по данным из памяти; детектор застревания (прыжок → стрейф → назад → перестроение). **Плавность «руки»**: скорость камеры набирается S-кривой (ограничен не только разгон, но и рывок `yaw_jerk_max`), мягкая мёртвая зона вместо порога, градусы→пиксели с переносом дробного остатка (мелкие повороты не дребезжат ±1 px), стартовая калибровка — плавный поворот-трапеция. На маршруте темп поворота камеры слегка плавает (±15 %), не одна фиксированная скорость. Переход «шёл → увидел мишень» не скачком: флик подхватывает текущую скорость и набирает её smoothstep ~0.1 с. **Мышь в игру уходит не одним событием на тик (40 Гц → «ступеньки» при 144+ fps), а размазывается фоновым потоком на под-шаги 500 Гц** (`EvdevInput`, как отчёты реальной мыши). Калибровка °/px берёт только тики с ровными px (во время флика оценка смещена задержкой ввода), |pitch| = |yaw| (одна сенса на обе оси). Случайность — не белый шум на каждый тик, а инерционный «ветер» как в [WindMouse](https://ben.land/post/2021/04/25/windmouse-human-mouse-movement/): AR(1)-вектор скорости, сильнее при движении, затухает у цели; блуждание взгляда на маршруте — плавный дрейф со скоростью-инерцией; тремора удержания по умолчанию нет (прицел стоит). **Мишени** (`nav.visible_enemies`): реакция 0.12–0.25 с → counter-strafe → **наводка как рука**: баллистический флик по профилю minimum-jerk (длительность по закону Фиттса: 30° ≈ 0.15 с, 90° ≈ 0.28 с, пик до ~720°/с) **по пологой дуге** (боковой изгиб sin πτ + sin 2πτ ≈ 2 % угла, плюс волна-«ветер» *положения* с огибающей sin πτ ≤ 1.5°, которая к концу маха гарантированно возвращается к нулю — если добавлять ветер к скорости, его интеграл оседает промахом), с feed-forward по смещению мишени, пока сами тормозим; конец чуть недоведён и с разбросом ~2.5 %; зрительная пауза 40–90 мс; остаток > 5° → корректирующее подвижение (второй короткий мах, точнее), мелкий остаток → экспоненциальная коррекция с S-образным набором скорости и торможением по остатку пути (|v| ≤ √(2a·e), без раскачки). **Точка прицеливания — верх груди (`target_aim_z` = 56u над ногами, `--aim-z`)**, не глаза: ошибка вверх остаётся в голове, вниз — в теле; высота своих глаз берётся из памяти (`m_vecViewOffset`). Допуск на выстрел раздельный: по yaw ≥1.0° / 80 % полуширины, по pitch ≥0.7° / 80 % запаса до верха головы (12u) — иначе по мишени на возвышении стреляли над головой. Как только прицел в допуске — tap-fire ЛКМ с интервалом 0.14–0.22 с **пока мишень не пропадёт** из видимых (или `shots_per_target`, если задан). Первый выстрел ≈ 0.3–0.5 с после появления. Мишень пропала → через `target_hold` возвращаемся к маршруту; мишень не пропадает / не навелись 6–8 с → идём дальше (её игнорируем 40 с). Стрельба включается только после того, как соглашение yaw подтверждено движением. С `--no-fire` — только взгляд (1.4–2.6 с), тело идёт дальше. Мишени — твёрдые препятствия: планировщик блокирует клетки под ними. Короткий обход — если он не разворачивает назад; иначе подходим на ~110u и стреляем с места (цель маршрута не меняется). После убийства сразу забываем мишень и продолжаем **исходную** дальнюю точку — не выбираем новую «самую дальнюю» из середины карты (иначе это стартовая сторона). Скрылась за углом — помним 45 с; место видно, а в списке нет — забыли, без «невидимого столба». |
| `simulator.py` | Физика Source (ускорение, трение, коллизии по карте, прыжок) + FakeReader/FakeInput → метрики: столкновения, min clearance, скорости поворота, застревания. Мишени-макеты: видимость по прямой на карте, твёрдый цилиндр, пропадают после `--hits-to-kill` попаданий и возвращаются через `--respawn` с; метрики плавности камеры (p95 ускорения, RMS рывка, смены направления в секунду, кривизна траектории прицела), наводки (время, ошибка прицела, пик скорости флика) и стрельбы (выстрелы/попадания по лучу камеры, задержка первого выстрела, время снятия мишени, скорость в момент выстрела). |
| `render.py` | PNG-радар на чистом Python: слои по высоте, маршрут, пройденный путь, направления взгляда, мишени (красные), линии взгляда на них и выстрелы (зелёные — попадания, жёлтые — промахи). |

Расширение карты — просто ещё одна запись (передать несколько файлов или склеить в один):

```bash
python3 python/tools/record_coords.py -m de_dust2 -l mid
python3 python/tools/merge_recordings.py -l tunels_mid data/recordings/de_dust2/tunels.json data/recordings/de_dust2/mid.json
python3 python/tools/walk_nav.py -r data/recordings/de_dust2/tunels_mid.json
```

## API

```python
from cs2_reader import Reader
from cs2_nav import CoordinateMap, Navigator, NavConfig
from cs2_control import create_input

reader = Reader.from_live()
pos = reader.local_position()
eye = reader.local_eye_angles()      # pitch, yaw
aim = reader.local_aim_point()       # координаты в мире

cmap = CoordinateMap.from_recording(["data/recordings/de_dust2/lower_tunel.json"])
nav = Navigator(reader, create_input(), cmap, NavConfig(loop=True))
nav.plan((pos.x, pos.y, pos.z))
nav.run(duration=120)
```
