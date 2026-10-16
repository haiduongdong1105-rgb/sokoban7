"""Chạy các map, mỗi map 1 tiến trình mới.
  python benchmark.py -t 30 --save baseline.json
  python benchmark.py -t 30 --compare baseline.json --maps 0-4,9"""
import argparse, json, subprocess, sys, time


def worker(i, tb):
    import solution
    from sokoban import PROBLEMS, SokobanState
    from validation import validate_solution
    p = PROBLEMS[i]
    mk = lambda: SokobanState("START", 0, None, p.width, p.height, p.robots,
                              p.boxes, p.storage, p.obstacles)
    s, t = mk(), time.perf_counter()
    try:
        final = solution.solve(s, timebound=tb)
        if (s.robots, s.boxes, s.gval, s.parent) != (p.robots, p.boxes, 0, None):
            r = dict(status="FAIL", steps=None, note="solve() sửa initial_state")
        else:
            ok, msg = (False, "trả False") if final is False else validate_solution(mk(), final)
            r = dict(status="PASS" if ok else "FAIL", steps=final.gval if ok else None, note=msg)
    except Exception as e:
        r = dict(status="CRASH", steps=None, note=repr(e)[:80])
    r["time"] = round(time.perf_counter() - t, 2)
    print("RESULT:" + json.dumps(r))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--worker", type=int)
    ap.add_argument("-t", type=float, default=120)
    ap.add_argument("--maps", default="0-19")
    ap.add_argument("--save")
    ap.add_argument("--compare")
    a = ap.parse_args()
    if a.worker is not None:
        return worker(a.worker, a.t)
    maps = [m for p in a.maps.split(",")
            for m in range(int(p.split("-")[0]), int(p.split("-")[-1]) + 1)]
    base = {}
    if a.compare:
        base = {r["map"]: r for r in json.load(open(a.compare))}
    out = []
    for m in maps:
        try:
            run = subprocess.run([sys.executable, __file__, "--worker", str(m), "-t", str(a.t)],
                                 capture_output=True, text=True, timeout=a.t + 5)
            line = [l for l in run.stdout.splitlines() if l.startswith("RESULT:")]
            r = json.loads(line[-1][7:]) if line else dict(
                status="CRASH", steps=None, time=0, note=run.stderr.strip()[-80:])
        except subprocess.TimeoutExpired:
            r = dict(status="TIMEOUT", steps=None, time=a.t + 5, note="")
        r["map"] = m
        b = base.get(m)
        extra = ""
        if b:
            extra = "  | base: %s %s" % (b["status"], b["steps"] or "")
            if b["status"] == "PASS" and r["status"] != "PASS":
                extra += "  !!! REGRESSION"
        print("map %2d  %-7s steps=%-4s %6.1fs %s%s" % (
            m, r["status"], r["steps"], r["time"], r["note"] if r["status"] != "PASS" else "", extra))
        out.append(r)
    print("PASS %d/%d, tổng bước %d" % (sum(r["status"] == "PASS" for r in out), len(out),
                                         sum(r["steps"] or 0 for r in out)))
    if a.save:
        json.dump(out, open(a.save, "w"), indent=1)


if __name__ == "__main__":
    main()
