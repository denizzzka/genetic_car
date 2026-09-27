#!/usr/bin/env bash
# Собирает libnewton/libdgCore/libdgPhysics/libdgNewtonAvx из исходников
# newton-dynamics с локальным патчем и кладёт их в корень пакета.
set -euo pipefail

NEWTON_URL="https://github.com/JulioJerez/newton-dynamics.git"
NEWTON_BRANCH="dliw/newton-dynamics"
# Коммит, на котором проверен патч 0001 (совпадает с tip на 2026-09-27).
NEWTON_PIN="597dd91d256b151c88e28eaf0eb435474ad9f414"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HERE="$ROOT/third_party/newton"
PATCH="$HERE/patches/0001-heightfield-bound-flip-passes.patch"
LIBS=(libnewton.so libdgCore.so libdgPhysics.so libdgNewtonAvx.so)

# Внешний checkout задаётся NEWTON_SRC, иначе берётся локальный кэш в репозитории.
SRC="${NEWTON_SRC:-$HERE/.cache/newton-dynamics}"
BUILD="$HERE/.cache/build"

if [ ! -d "$SRC/.git" ]; then
	echo "клонирую $NEWTON_URL -> $SRC"
	mkdir -p "$(dirname "$SRC")"
	git clone --quiet "$NEWTON_URL" "$SRC"
fi

git -C "$SRC" fetch --quiet --depth=1 origin "$NEWTON_BRANCH" || true
git -C "$SRC" checkout --quiet --force "$NEWTON_PIN"
git -C "$SRC" apply --check "$PATCH" 2>/dev/null || git -C "$SRC" apply "$PATCH"
echo "патч применён к $NEWTON_PIN"

cmake -S "$SRC/newton-3.14" -B "$BUILD" -G "Unix Makefiles" \
	-DCMAKE_BUILD_TYPE=Release \
	-DNEWTON_BUILD_SHARED_LIBS=ON \
	-DNEWTON_BUILD_SANDBOX_DEMOS=OFF \
	-DNEWTON_BUILD_PROFILER=OFF >/dev/null
cmake --build "$BUILD" -j"$(nproc)" --target newton dgCore dgPhysics dgNewtonAvx >/dev/null

for lib in "${LIBS[@]}"; do
	install -m 644 "$BUILD/lib/$lib" "$ROOT/$lib"
	echo "установлен $lib"
done

echo
echo "ВАЖНО: dub build копирует в корень пакета оригинальные .so из dagon:newton"
echo "и затирает эти. После dub build запусти скрипт заново, иначе вернётся"
echo "непропатченная libdgPhysics.so."
