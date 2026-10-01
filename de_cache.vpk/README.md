# de_cache.vpk

Достаёт `maps/de_cache.nav` из архива `data/de_cache.vpk` и печатает зоны, координаты углов, связи, лестницы и параметры генерации.

```bash
python3 decompile_nav.py
```

Результат пишется в `out/de_cache.nav` и `out/de_cache.nav.json`.

Разбор каталога VPK и формата `.nav` (версии 30–36, блоки KV3 в версии 36) сделан по коду Source 2 Viewer в `ValveResourceFormat/`.

Powered by [Source 2 Viewer](https://s2v.app) ([ValveResourceFormat](https://github.com/ValveResourceFormat/ValveResourceFormat)).
