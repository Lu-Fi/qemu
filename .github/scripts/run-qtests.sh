#!/bin/sh
# Run every tests/qtest/ingenic-*-test in BUILD_DIR against its
# qemu-system-mipsel, JOBS at a time, each under a TIMEOUT-second limit.
# Per-test logs go to LOGS; exits 1 and names the tests if any failed.
# usage: run-qtests.sh BUILD_DIR LOGS
set -u
build=$(cd "$1" && pwd)
logs=$2
: "${JOBS:=$(nproc)}"
: "${TIMEOUT:=600}"
mkdir -p "$logs"
logs=$(cd "$logs" && pwd)
export QTEST_QEMU_BINARY="$build/qemu-system-mipsel" build logs TIMEOUT

# the test sources are the list, so a new test needs no edit here
names=$(cd "$(dirname "$0")/../../tests/qtest" && ls ingenic-*-test.c | sed 's/\.c$//')

(cd "$build" && ninja $(for t in $names; do echo "tests/qtest/$t"; done)) || exit 1

rm -f "$logs"/*.fail
printf '%s\n' $names | xargs -P "$JOBS" -I{} sh -c '
  s=$(date +%s)
  timeout "$TIMEOUT" "$build/tests/qtest/{}" --tap -k > "$logs/{}.log" 2>&1
  rc=$?
  echo "$rc $(($(date +%s) - s))s {}"
  [ "$rc" -eq 0 ] || echo "$rc" > "$logs/{}.fail"
'
n=$(ls "$logs"/*.fail 2>/dev/null | wc -l)
if [ "$n" -ne 0 ]; then
  echo "FAILED ($n):"
  for f in "$logs"/*.fail; do basename "$f" .fail; done
  exit 1
fi
echo "all $(echo "$names" | wc -l) qtests passed"
