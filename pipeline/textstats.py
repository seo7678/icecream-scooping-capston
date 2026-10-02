"""글 한 편에서 스타일 수치를 뽑는다. 표준 라이브러리만 쓴다.

한국어 종결어미 분류는 정규식 근사라서 정확하지 않다.
절대값보다 "내 글끼리 / AI 글과의 상대 비교"에 쓴다.
"""
import re
import statistics

EMOJI = re.compile(
    "[\U0001F300-\U0001FAFF\U00002600-\U000027BF\U0001F000-\U0001F2FF⭐⬆✅❌]"
)
CONNECTIVES = [
    "그리고", "하지만", "그러나", "따라서", "또한", "한편", "그래서", "그런데", "즉",
    "결국", "예를 들어", "결론적으로", "나아가", "반면", "특히", "이를 통해", "더불어",
]
ENDINGS = [  # 위에서부터 먼저 맞는 것으로 분류
    ("합쇼체(~니다)", re.compile(r"(니다|니까|십시오|세요)$")),
    ("해요체(~요)", re.compile(r"요$")),
    ("평서 ~다", re.compile(r"다$")),
    ("~음/함/됨체", re.compile(r"[음함됨임슴]$")),
    ("반말(~어/야/지/네)", re.compile(r"[어아야지네냐게군죠자]$")),
]
LIST_LINE = re.compile(r"^\s*([-*•·]|\d+[.)])\s+")
HEADER_LINE = re.compile(r"^\s*#{1,6}\s+")
TABLE_LINE = re.compile(r"^\s*\|.*\|\s*$")
CODE_FENCE = re.compile(r"```.*?```", re.S)


def strip_code(text):
    return CODE_FENCE.sub("", text)


def split_sentences(text):
    """줄바꿈과 문장부호 기준 근사 분리. 목록 기호는 떼고 센다."""
    out = []
    for line in text.splitlines():
        line = HEADER_LINE.sub("", LIST_LINE.sub("", line)).strip()
        if not line or TABLE_LINE.match(line):
            continue
        for s in re.split(r"(?<=[.!?…])\s+", line):
            s = s.strip()
            if s:
                out.append(s)
    return out


def split_paragraphs(text):
    return [p for p in re.split(r"\n\s*\n", text) if p.strip()]


def classify_ending(sentence):
    core = re.sub(r"[\s.!?…~ㅋㅎ\"')\]]+$", "", sentence)
    if not core:
        return "기호/기타"
    for name, rx in ENDINGS:
        if rx.search(core):
            return name
    return "체언/기타"


def per_k(count, chars):
    return round(count * 1000 / chars, 2) if chars else 0.0


def analyze(text):
    text = strip_code(text)
    chars = len(re.sub(r"\s", "", text))
    lines = [l for l in text.splitlines() if l.strip()]
    sents = split_sentences(text)
    lens = [len(s.split()) for s in sents] or [0]
    paras = split_paragraphs(text)
    para_sents = [len(split_sentences(p)) for p in paras] or [0]

    endings = {}
    for s in sents:
        k = classify_ending(s)
        endings[k] = endings.get(k, 0) + 1
    n = max(len(sents), 1)

    words = re.findall(r"[A-Za-z][A-Za-z0-9_\-]*", text)
    nl = max(len(lines), 1)
    conn = {c: text.count(c) for c in CONNECTIVES if text.count(c)}

    mean_len = statistics.mean(lens)
    sd_len = statistics.pstdev(lens) if len(lens) > 1 else 0.0
    return {
        "chars": chars,
        "sentences": len(sents),
        "sent_len_mean": round(mean_len, 2),
        "sent_len_sd": round(sd_len, 2),
        # 문장 길이가 들쭉날쭉한 정도. AI 글은 보통 낮다
        "sent_len_burst": round(sd_len / mean_len, 3) if mean_len else 0.0,
        "para_sents_mean": round(statistics.mean(para_sents), 2),
        "endings": {k: round(v / n, 3) for k, v in sorted(endings.items())},
        "connectives_per_k": per_k(sum(conn.values()), chars),
        "connectives": conn,
        "list_line_ratio": round(sum(bool(LIST_LINE.match(l)) for l in lines) / nl, 3),
        "header_line_ratio": round(sum(bool(HEADER_LINE.match(l)) for l in lines) / nl, 3),
        "bold_per_k": per_k(len(re.findall(r"\*\*[^*\n]+\*\*", text)), chars),
        "emdash_per_k": per_k(text.count("—") + text.count("–"), chars),
        "emoji_per_k": per_k(len(EMOJI.findall(text)), chars),
        "laugh_per_k": per_k(len(re.findall(r"[ㅋㅎ]{2,}|ㅠ{2,}|ㅜ{2,}", text)), chars),
        "excl_per_k": per_k(text.count("!"), chars),
        "quest_per_k": per_k(text.count("?"), chars),
        "paren_per_k": per_k(text.count("(") , chars),
        # 구체성 대리 지표: 숫자, 영문 단어
        "digit_per_k": per_k(len(re.findall(r"\d+(?:\.\d+)?", text)), chars),
        "latin_word_per_k": per_k(len(words), chars),
    }


# 파일 여러 개를 합칠 때 평균낼 스칼라 지표
SCALARS = [
    "sent_len_mean", "sent_len_sd", "sent_len_burst", "para_sents_mean",
    "connectives_per_k", "list_line_ratio", "header_line_ratio", "bold_per_k",
    "emdash_per_k", "emoji_per_k", "laugh_per_k", "excl_per_k", "quest_per_k",
    "paren_per_k", "digit_per_k", "latin_word_per_k",
]


def aggregate(per_file):
    """파일별 분석 결과 리스트 -> 지표별 mean/sd/min/max, 종결어미 평균."""
    agg = {"files": len(per_file), "scalars": {}, "endings": {}}
    for k in SCALARS:
        vals = [r[k] for r in per_file]
        agg["scalars"][k] = {
            "mean": round(statistics.mean(vals), 3),
            "sd": round(statistics.pstdev(vals), 3) if len(vals) > 1 else 0.0,
            "min": min(vals),
            "max": max(vals),
        }
    names = sorted({e for r in per_file for e in r["endings"]})
    for e in names:
        agg["endings"][e] = round(statistics.mean(r["endings"].get(e, 0) for r in per_file), 3)
    return agg
