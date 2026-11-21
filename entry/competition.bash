#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$ROOT/ground_straight_nav/src${PYTHONPATH:+:$PYTHONPATH}"
MODE="${1:-}"
if [[ "$MODE" == prepare || "$MODE" == check ]]; then
  LOG="$(mktemp "$ROOT/preflight_${MODE}.XXXXXX.log")"
  # Each attempt keeps its own evidence; retain the conventional latest copy.
  trap 'cp -- "$LOG" "$ROOT/preflight_${MODE}.log"' EXIT
  if ! python3 "$ROOT/verify.py" >"$LOG" 2>&1; then
    printf 'PRECHECK_FAILED stage=entry_integrity log=%s\n' "$LOG" >&2
    exit 2
  fi
  if ! source "$ROOT/../flight/env.bash" >>"$LOG" 2>&1; then
    printf 'PRECHECK_FAILED stage=environment_setup log=%s\n' "$LOG" >&2
    exit 2
  fi
  set +e
  python3 "$ROOT/competition.py" "$@" 2>&1 | while IFS= read -r LINE; do
    printf '%s\n' "$LINE" >>"$LOG"
    case "$LINE" in
      PRECHECK_WAIT*) printf '%s\n' "$LINE" >&2 ;;
      TARGET_PROMPT:*) printf '%s\n' "${LINE#TARGET_PROMPT:}" >&2 ;;
    esac
  done
  STATUS=${PIPESTATUS[0]}
  set -e
  if [[ $STATUS -eq 0 ]]; then
    SUMMARY="$(grep '^PRECHECK_READY' "$LOG" | tail -n 1 || true)"
    [[ -n "$SUMMARY" ]] || SUMMARY="PRECHECK_READY stage=$MODE"
    printf '%s\n' "$SUMMARY"
    if [[ "$MODE" == prepare ]]; then
      grep '^NEXT_COMMAND=' "$LOG" | tail -n 1 || true
    fi
  else
    REASON="$(grep -E '^(ValueError|RuntimeError|OSError|PREPARE_CLASS)[:=]' "$LOG" | tail -n 1 || true)"
    [[ -n "$REASON" ]] || REASON="stage=$MODE failed"
    printf 'PRECHECK_FAILED %s log=%s\n' "$REASON" "$LOG" >&2
  fi
  exit "$STATUS"
fi
python3 "$ROOT/verify.py" >/dev/null
source "$ROOT/../flight/env.bash"
exec python3 "$ROOT/competition.py" "$@"
