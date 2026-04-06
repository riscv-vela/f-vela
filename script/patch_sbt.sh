#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

cd $CHIPYARD_DIR
echo "Patching build.sbt with F-Vela config..."
diff -u build.sbt generators/_f_vela/script/f-vela_build.sbt > generators/_f_vela/script/chipyard_vela.patch
echo "Generated patch file at: $CHIPYARD_DIR/generators/_f_vela/script/chipyard_vela.patch"
# patch -p0 < "generators/_f_vela/script/chipyard_vela.patch"
