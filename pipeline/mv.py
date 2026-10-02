#!/usr/bin/env python3
"""my-voice CLI.

  mv.py stats FILE...                      글 수치 보기
  mv.py slop FILE                          AI 티 패턴 검사
  mv.py profile DOMAIN FORMAT              corpus/DOMAIN/FORMAT/ 으로 프로파일 수치 생성
  mv.py compare DOMAIN FORMAT FILE         프로파일 대비 FILE이 벗어난 지표
  mv.py feedback add|report ...            AI 초안 vs 내 최종본 기록·집계
  mv.py blind make|score ...               블라인드 테스트
  mv.py kakao CHAT.txt --name 내이름       카톡 내보내기에서 내 말만 추출
"""
import argparse
import collections
import datetime
import difflib
import json
import os
import random
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import slopcheck  # noqa: E402
import textstats  # noqa: E402

ROOT = Path(os.environ.get("MV_DATA_ROOT") or Path(__file__).resolve().parent.parent)  # 테스트에서 덮어씀
TEXT_EXT = {".md", ".txt"}


def read(p):
    return Path(p).read_text(encoding="utf-8")


def corpus_files(domain, fmt):
    d = ROOT / "corpus" / domain / fmt
    if not d.is_dir():
        sys.exit(f"없음: {d}")
    files = sorted(p for p in d.iterdir()
                   if p.suffix in TEXT_EXT and not p.name.endswith(".meta.md") and p.name != "README.md")
    if not files:
        sys.exit(f"글이 없음: {d}")
    return files


# ---------- stats / slop ----------
def cmd_stats(a):
    for f in a.files:
        st = textstats.analyze(read(f))
        if a.json:
            print(json.dumps(st, ensure_ascii=False, indent=2))
            continue
        print(f"== {f}")
        for k, v in st.items():
            if k not in ("endings", "connectives"):
                print(f"  {k:<20} {v}")
        print("  endings:", {k: f"{v:.0%}" for k, v in st["endings"].items()})
        print("  connectives:", st["connectives"])


def cmd_slop(a):
    res = slopcheck.check(read(a.file))
    if a.json:
        print(json.dumps(res, ensure_ascii=False, indent=2))
    else:
        print(slopcheck.format_report(res))
    if a.max_score is not None and res["score_per_k"] > a.max_score:
        sys.exit(1)


# ---------- profile ----------
def ngram_top(counter, n=8):
    return [f"{k} ({v})" for k, v in counter.most_common(n)]


def cmd_profile(a):
    files = corpus_files(a.domain, a.format)
    texts = [read(f) for f in files]
    per = [textstats.analyze(t) for t in texts]
    agg = textstats.aggregate(per)
    agg.update(domain=a.domain, format=a.format,
               built=datetime.date.today().isoformat(), source=[f.name for f in files])

    first_sents, last_sents = [], []
    for t in texts:
        ss = textstats.split_sentences(textstats.strip_code(t))
        if ss:
            first_sents.append(ss[0])
            last_sents.append(ss[-1])
    allsents = [s for t in texts for s in textstats.split_sentences(textstats.strip_code(t))]
    sent_openers = collections.Counter(s.split()[0] for s in allsents if len(s.split()) > 3)
    endings2 = collections.Counter(re.sub(r"[\s.!?…~]+$", "", s)[-2:] for s in allsents)
    med = agg["scalars"]["sent_len_mean"]["mean"]
    typical = sorted(allsents, key=lambda s: abs(len(s.split()) - med))[:8]

    agg["candidates"] = {
        "글 첫 문장": first_sents[:8],
        "글 마지막 문장": last_sents[:8],
        "문장 첫 어절 TOP": ngram_top(sent_openers),
        "문장 끝 두 글자 TOP": ngram_top(endings2),
        "평균 길이에 가까운 문장": typical,
    }
    out = ROOT / "profiles" / f"{a.domain}_{a.format}.stats.json"
    out.write_text(json.dumps(agg, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"저장: {out.relative_to(ROOT)}  (글 {len(files)}편)")
    if len(files) < 5:
        print("주의: 5편 미만이라 수치 편차가 믿을 만하지 않다")
    sc = agg["scalars"]
    for k in ("sent_len_mean", "sent_len_burst", "list_line_ratio", "connectives_per_k", "digit_per_k"):
        print(f"  {k:<18} 평균 {sc[k]['mean']}  (편 간 sd {sc[k]['sd']}, 범위 {sc[k]['min']}~{sc[k]['max']})")
    md = out.with_suffix("").with_suffix(".md")
    if not md.exists():
        tpl = (ROOT / "profiles" / "_TEMPLATE.md").read_text(encoding="utf-8")
        md.write_text(tpl.replace("<분야> / <형태>", f"{a.domain} / {a.format}"), encoding="utf-8")
        print(f"템플릿 생성: {md.relative_to(ROOT)}  (규칙·예문은 직접/스킬로 채움)")


# ---------- compare ----------
def cmd_compare(a):
    prof = json.loads((ROOT / "profiles" / f"{a.domain}_{a.format}.stats.json").read_text(encoding="utf-8"))
    st = textstats.analyze(read(a.file))
    rows = []
    for k, ref in prof["scalars"].items():
        # 글이 적을 때 sd가 0에 가까우므로 평균의 20%를 하한으로 둔다
        floor = 0.05 if k.endswith(("_ratio", "_burst")) else 1.0
        tol = max(ref["sd"], abs(ref["mean"]) * 0.2, floor)
        z = (st[k] - ref["mean"]) / tol
        rows.append((abs(z), k, st[k], ref["mean"], z))
    rows.sort(reverse=True)
    print(f"{a.file}  vs  {a.domain}/{a.format} (글 {prof['files']}편)")
    off = [r for r in rows if r[0] > 1.5]
    for _, k, v, m, z in (off or rows[:3]):
        arrow = "높음" if z > 0 else "낮음"
        print(f"  {k:<20} 이 글 {v:<8} 내 평균 {m:<8} -> {arrow} ({min(abs(z), 99):.1f}배 허용폭)")
    if not off:
        print("  (모든 지표가 내 범위 안. 위는 그나마 가장 먼 3개)")
    for e, ref in prof["endings"].items():
        v = st["endings"].get(e, 0)
        if abs(v - ref) > 0.25:
            print(f"  종결어미 {e}: 이 글 {v:.0%} / 내 평균 {ref:.0%}")
    sys.exit(1 if len(off) >= 4 else 0)


# ---------- feedback ----------
def sent_set(t):
    return textstats.split_sentences(textstats.strip_code(t))


def diff_stats(draft, final):
    char_ratio = difflib.SequenceMatcher(None, draft, final, autojunk=False).ratio()
    ds, fs = sent_set(draft), sent_set(final)
    sm = difflib.SequenceMatcher(None, ds, fs, autojunk=False)
    removed, added = [], []
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag in ("delete", "replace"):
            removed += ds[i1:i2]
        if tag in ("insert", "replace"):
            added += fs[j1:j2]
    return {"change_ratio": round(1 - char_ratio, 3), "draft_sents": len(ds), "final_sents": len(fs),
            "removed": removed, "added": added}


def cmd_feedback(a):
    base = ROOT / "feedback"
    if a.action == "add":
        draft, final = read(a.draft), read(a.final)
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        d = base / f"{stamp}_{a.domain}_{a.format}"
        d.mkdir(parents=True)
        (d / "draft.md").write_text(draft, encoding="utf-8")
        (d / "final.md").write_text(final, encoding="utf-8")
        res = diff_stats(draft, final)
        res.update(domain=a.domain, format=a.format, note=a.note or "",
                   draft_slop=slopcheck.check(draft)["score_per_k"],
                   final_slop=slopcheck.check(final)["score_per_k"])
        (d / "diff.json").write_text(json.dumps(res, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"저장: {d.relative_to(ROOT)}")
        print(f"  변경 비율 {res['change_ratio']:.0%}  삭제 문장 {len(res['removed'])}  추가 문장 {len(res['added'])}")
        print(f"  슬롭 점수(1000자당) 초안 {res['draft_slop']} -> 최종 {res['final_slop']}")
        return
    runs = [json.loads(p.read_text(encoding="utf-8")) for p in sorted(base.glob("*/diff.json"))]
    if a.domain:
        runs = [r for r in runs if r["domain"] == a.domain and (not a.format or r["format"] == a.format)]
    if not runs:
        sys.exit("feedback 기록 없음")
    print(f"기록 {len(runs)}건, 평균 변경 비율 {sum(r['change_ratio'] for r in runs)/len(runs):.0%}")
    print("추이:", " ".join(f"{r['change_ratio']:.0%}" for r in runs))
    rem = collections.Counter(w for r in runs for s in r["removed"] for w in set(re.findall(r"[가-힣]{2,}", s)))
    add = collections.Counter(w for r in runs for s in r["added"] for w in set(re.findall(r"[가-힣]{2,}", s)))
    print("내가 자주 지운 단어(슬롭 후보):", ngram_top(rem, 12))
    print("내가 자주 넣은 단어(내 어휘 후보):", ngram_top(add, 12))


# ---------- blind ----------
def cmd_blind(a):
    if a.action == "make":
        items = []
        for label, paths in (("mine", a.mine), ("ai", a.ai), ("system", a.system)):
            items += [(label, p) for p in paths or []]
        random.Random(a.seed).shuffle(items)
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        key, doc = {}, ["# 블라인드 테스트\n", "각 글이 (A) 내가 쓴 것 (B) AI가 그냥 쓴 것 (C) 내 스타일로 만든 것 중 무엇인지 적어라. 읽는 순서대로, 뒤로 가지 않는다.\n"]
        for i, (label, p) in enumerate(items, 1):
            key[f"Q{i}"] = {"label": label, "src": str(p)}
            doc.append(f"\n## Q{i}\n\n{read(p).strip()}\n\n답:\n")
        (out / "blind_test.md").write_text("\n".join(doc), encoding="utf-8")
        (out / "answer_key.json").write_text(json.dumps(key, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"생성: {out}/blind_test.md ({len(items)}문항). answer_key.json 은 풀기 전에 열지 말 것")
        return
    key = json.loads(read(a.key))
    ans = json.loads(read(a.answers))  # {"Q1": "mine"|"ai"|"system"}
    by = collections.defaultdict(lambda: [0, 0])
    for q, meta in key.items():
        by[meta["label"]][1] += 1
        by[meta["label"]][0] += ans.get(q) == meta["label"]
    for label, (ok, n) in by.items():
        print(f"{label:<7} 맞힌 {ok}/{n}")
    sys_items = [q for q, m in key.items() if m["label"] == "system"]
    if sys_items:
        fooled = sum(ans.get(q) == "mine" for q in sys_items)
        print(f"시스템 출력을 '내 글'로 본 비율: {fooled}/{len(sys_items)}")


# ---------- kakao ----------
KAKAO_PC = re.compile(r"^(\d{4})[.년] ?(\d{1,2})[.월] ?(\d{1,2})[.일]? ?(오전|오후) (\d{1,2}):(\d{2}),? (.+?) : (.*)$")
KAKAO_MOBILE = re.compile(r"^\[(.+?)\] \[(오전|오후) (\d{1,2}):(\d{2})\] (.*)$")
KAKAO_DAY = re.compile(r"^-+ (\d{4})년 (\d{1,2})월 (\d{1,2})일")
SKIP_MSG = re.compile(r"^(사진|동영상|이모티콘|삭제된 메시지입니다\.?|파일: .*|https?://\S+|#\S+)$")


def cmd_kakao(a):
    lines = read(a.file).splitlines()
    msgs, cur, day = [], None, None
    for ln in lines:
        m = KAKAO_PC.match(ln)
        if m:
            y, mo, d, ap, h, mi, who, text = m.groups()
            h = int(h) % 12 + (12 if ap == "오후" else 0)
            cur = {"t": datetime.datetime(int(y), int(mo), int(d), h, int(mi)), "who": who, "text": text}
            msgs.append(cur)
            continue
        m = KAKAO_DAY.match(ln)
        if m:
            day = tuple(map(int, m.groups()))
            cur = None
            continue
        m = KAKAO_MOBILE.match(ln)
        if m and day:
            who, ap, h, mi, text = m.groups()
            h = int(h) % 12 + (12 if ap == "오후" else 0)
            cur = {"t": datetime.datetime(*day, h, int(mi)), "who": who, "text": text}
            msgs.append(cur)
            continue
        if cur is not None and ln.strip():
            cur["text"] += "\n" + ln  # 여러 줄 메시지의 이어지는 줄
    mine = [m for m in msgs if m["who"] == a.name and not SKIP_MSG.match(m["text"].strip())]
    if not mine:
        names = collections.Counter(m["who"] for m in msgs).most_common(5)
        sys.exit(f"'{a.name}' 메시지 없음. 대화에 보이는 이름: {names}")
    # 같은 사람이 gap분 안에 연달아 보낸 메시지는 한 덩어리(턴)로 묶는다
    turns, buf, last = [], [], None
    for m in mine:
        if last and (m["t"] - last).total_seconds() > a.gap * 60:
            turns.append(buf)
            buf = []
        buf.append(m["text"].strip())
        last = m["t"]
    turns.append(buf)
    turns = [t for t in turns if sum(len(x) for x in t) >= a.min_chars]
    out = Path(a.out) if a.out else ROOT / "corpus" / a.domain / "kakao" / (Path(a.file).stem + ".md")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n\n".join("\n".join(t) for t in turns) + "\n", encoding="utf-8")
    print(f"내 메시지 {len(mine)}개 -> 턴 {len(turns)}개 (최소 {a.min_chars}자), 저장 {out}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("stats"); p.add_argument("files", nargs="+"); p.add_argument("--json", action="store_true"); p.set_defaults(fn=cmd_stats)
    p = sub.add_parser("slop"); p.add_argument("file"); p.add_argument("--json", action="store_true")
    p.add_argument("--max-score", type=float, help="1000자당 점수가 이보다 크면 종료코드 1"); p.set_defaults(fn=cmd_slop)
    p = sub.add_parser("profile"); p.add_argument("domain"); p.add_argument("format"); p.set_defaults(fn=cmd_profile)
    p = sub.add_parser("compare"); p.add_argument("domain"); p.add_argument("format"); p.add_argument("file"); p.set_defaults(fn=cmd_compare)

    p = sub.add_parser("feedback"); fs = p.add_subparsers(dest="action", required=True)
    q = fs.add_parser("add"); q.add_argument("domain"); q.add_argument("format")
    q.add_argument("--draft", required=True); q.add_argument("--final", required=True); q.add_argument("--note")
    q = fs.add_parser("report"); q.add_argument("--domain"); q.add_argument("--format")
    p.set_defaults(fn=cmd_feedback)

    p = sub.add_parser("blind"); bs = p.add_subparsers(dest="action", required=True)
    q = bs.add_parser("make"); q.add_argument("--mine", nargs="+"); q.add_argument("--ai", nargs="+")
    q.add_argument("--system", nargs="+"); q.add_argument("--out", required=True); q.add_argument("--seed", type=int, default=None)
    q = bs.add_parser("score"); q.add_argument("key"); q.add_argument("answers")
    p.set_defaults(fn=cmd_blind)

    p = sub.add_parser("kakao"); p.add_argument("file"); p.add_argument("--name", required=True)
    p.add_argument("--domain", default="personal"); p.add_argument("--out")
    p.add_argument("--gap", type=int, default=5, help="분. 이 안에 연달아 보낸 메시지는 한 덩어리"); p.add_argument("--min-chars", type=int, default=15)
    p.set_defaults(fn=cmd_kakao)

    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
