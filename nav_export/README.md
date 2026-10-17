# nav_export

Достаёт навигационную сетку (`maps/<карта>.nav`) из VPK и пишет разбор в `data/maps`.

```bash
python3 nav_export/decompile_nav.py data/de_cache.vpk
```

Результат: `data/maps/de_cache.nav` и `data/maps/de_cache.nav.json`.

Разбор каталога VPK и формата `.nav` (версии 30–36, блоки KV3 в версии 36) сделан по коду Source 2 Viewer в `ValveResourceFormat/`.

Powered by [Source 2 Viewer](https://s2v.app) ([ValveResourceFormat](https://github.com/ValveResourceFormat/ValveResourceFormat)).
