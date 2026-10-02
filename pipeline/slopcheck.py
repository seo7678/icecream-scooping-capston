"""AI 티 나는 패턴 검사. slop/patterns.json 을 정규식으로 돌린다."""
import json
import re
import statistics
from pathlib import Path

import textstats

ROOT = Path(__file__).resolve().parent.parent
PATTERNS = ROOT / "slop" / "patterns.json"
ALLOW = ROOT / "slop" / "allow.txt"


def load_patterns():
    data = json.loads(PATTERNS.read_text(encoding="utf-8"))
    return [dict(p, _rx=re.compile(p["rx"])) for p in data["patterns"]]


def load_allow():
    if not ALLOW.exists():
        return []
    return [l.strip() for l in ALLOW.read_text(encoding="utf-8").splitlines()
            if l.strip() and not l.lstrip().startswith("#")]


def check(text, patterns=None, allow=None):
    patterns = patterns or load_patterns()
    allow = load_allow() if allow is None else allow
    body = textstats.strip_code(text)
    hits = []
    for no, line in enumerate(body.splitlines(), 1):
        if not line.strip() or any(a in line for a in allow):
            continue
        for p in patterns:
            for m in p["_rx"].finditer(line):
                hits.append({"line": no, "id": p["id"], "cat": p["cat"], "w": p["w"],
                             "match": m.group(0).strip(), "note": p["note"]})
    chars = len(re.sub(r"\s", "", body))
    score = sum(h["w"] for h in hits)
    return {
        "chars": chars,
        "hits": hits,
        "score": score,
        "score_per_k": round(score * 1000 / chars, 2) if chars else 0.0,
        "flags": structural_flags(body),
    }


def structural_flags(text):
    """정규식으로 못 잡는 구조 신호."""
    st = textstats.analyze(text)
    flags = []
    if st["sentences"] >= 8 and st["sent_len_burst"] < 0.35:
        flags.append(f"문장 길이가 너무 균일함 (변동 {st['sent_len_burst']})")
    if st["list_line_ratio"] > 0.4:
        flags.append(f"불릿/번호 줄 비율 {st['list_line_ratio']:.0%}")
    if st["header_line_ratio"] > 0.15:
        flags.append(f"헤더 줄 비율 {st['header_line_ratio']:.0%}")
    if st["bold_per_k"] > 6:
        flags.append(f"굵은 글씨 {st['bold_per_k']}회/1000자")
    if st["chars"] >= 300 and st["digit_per_k"] == 0:
        flags.append("숫자가 하나도 없음 (구체성 부족)")
    paras = textstats.split_paragraphs(text)
    if len(paras) >= 4:
        sizes = [len(textstats.split_sentences(p)) for p in paras]
        if statistics.pstdev(sizes) < 0.5:
            flags.append("문단 크기가 모두 비슷함")
    return flags


def format_report(res):
    out = [f"슬롭 점수 {res['score']} (1000자당 {res['score_per_k']}), 적중 {len(res['hits'])}건"]
    for h in res["hits"]:
        out.append(f"  L{h['line']:<3} [{h['cat']}/{h['id']}] \"{h['match']}\"  - {h['note']}")
    for f in res["flags"]:
        out.append(f"  구조: {f}")
    return "\n".join(out)
