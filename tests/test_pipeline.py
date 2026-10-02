import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
MV = str(REPO / "pipeline" / "mv.py")
FIX = REPO / "tests" / "fixtures"
sys.path.insert(0, str(REPO / "pipeline"))
import slopcheck  # noqa: E402
import textstats  # noqa: E402


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp())
        (self.tmp / "profiles").mkdir()
        shutil.copy(REPO / "profiles" / "_TEMPLATE.md", self.tmp / "profiles")
        self.env = dict(os.environ, MV_DATA_ROOT=str(self.tmp))

    def tearDown(self):
        shutil.rmtree(self.tmp)

    def mv(self, *args, ok=True):
        r = subprocess.run([sys.executable, MV, *map(str, args)], capture_output=True, text=True, env=self.env)
        if ok:
            self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        return r


class TestSlop(unittest.TestCase):
    def test_ai_scores_higher_than_human(self):
        h = slopcheck.check((FIX / "human_like.md").read_text(encoding="utf-8"))
        a = slopcheck.check((FIX / "ai_like.md").read_text(encoding="utf-8"))
        self.assertEqual(h["score"], 0)
        self.assertGreater(a["score_per_k"], 20)
        self.assertTrue(a["flags"])

    def test_allowlist(self):
        t = "결론적으로 이건 내 말버릇이다."
        self.assertGreater(slopcheck.check(t, allow=[])["score"], 0)
        self.assertEqual(slopcheck.check(t, allow=["결론적으로"])["score"], 0)

    def test_code_block_ignored(self):
        t = "```\n결론적으로 이를 통해\n```\n로드셀 3 N."
        self.assertEqual(slopcheck.check(t, allow=[])["score"], 0)


class TestStats(unittest.TestCase):
    def test_endings_and_digits(self):
        st = textstats.analyze("로드셀을 바꿨다. 값이 안정됨. 이거 맞아요? 3 N 기준입니다.")
        self.assertIn("합쇼체(~니다)", st["endings"])
        self.assertIn("해요체(~요)", st["endings"])
        self.assertGreater(st["digit_per_k"], 0)

    def test_empty(self):
        st = textstats.analyze("")
        self.assertEqual(st["chars"], 0)


class TestCLI(Base):
    def make_corpus(self, n=3):
        d = self.tmp / "corpus" / "capstone" / "log"
        d.mkdir(parents=True)
        for i in range(n):
            shutil.copy(FIX / "human_like.md", d / f"2026-09-0{i+1}_x.md")

    def test_profile_and_compare(self):
        self.make_corpus()
        self.mv("profile", "capstone", "log")
        prof = json.loads((self.tmp / "profiles" / "capstone_log.stats.json").read_text(encoding="utf-8"))
        self.assertEqual(prof["files"], 3)
        self.assertTrue((self.tmp / "profiles" / "capstone_log.md").exists())
        same = self.mv("compare", "capstone", "log", FIX / "human_like.md")
        self.assertIn("범위 안", same.stdout)
        far = self.mv("compare", "capstone", "log", FIX / "ai_like.md", ok=False)
        self.assertEqual(far.returncode, 1)
        self.assertIn("header_line_ratio", far.stdout)

    def test_feedback_roundtrip(self):
        self.mv("feedback", "add", "capstone", "log", "--draft", FIX / "ai_like.md", "--final", FIX / "human_like.md")
        r = self.mv("feedback", "report")
        self.assertIn("기록 1건", r.stdout)

    def test_blind(self):
        out = self.tmp / "b"
        self.mv("blind", "make", "--mine", FIX / "human_like.md", "--ai", FIX / "ai_like.md", "--out", out, "--seed", 1)
        key = json.loads((out / "answer_key.json").read_text(encoding="utf-8"))
        ans = out / "ans.json"
        ans.write_text(json.dumps({q: m["label"] for q, m in key.items()}), encoding="utf-8")
        r = self.mv("blind", "score", out / "answer_key.json", ans)
        self.assertIn("맞힌 1/1", r.stdout)

    def test_kakao_mobile_and_pc(self):
        for name in ("kakao_mobile.txt", "kakao_pc.txt"):
            out = self.tmp / (name + ".out.md")
            self.mv("kakao", FIX / name, "--name", "서원", "--out", out, "--min-chars", 5)
            text = out.read_text(encoding="utf-8")
            self.assertIn("저울만 있으면 될듯", text)  # 여러 줄 메시지 이어붙임
            self.assertNotIn("5시", text)  # 남의 말 제외
            self.assertNotIn("삭제된 메시지", text)
            self.assertNotIn("사진", text)

    def test_kakao_wrong_name(self):
        r = self.mv("kakao", FIX / "kakao_pc.txt", "--name", "없는사람", "--out", self.tmp / "o.md", ok=False)
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("민수", r.stderr)


if __name__ == "__main__":
    unittest.main()
