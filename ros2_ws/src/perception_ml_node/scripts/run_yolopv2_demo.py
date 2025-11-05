#!/usr/bin/env python3
import argparse, subprocess, sys, os
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='yolopv2/weights/yolopv2.pt')
    ap.add_argument('--inputs', nargs='+', default=['assets/img1.png'])
    ap.add_argument('--outdir', default='outputs')
    ap.add_argument('--imgw', type=int, default=640)
    ap.add_argument('--imgh', type=int, default=384)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    repo_dir = Path(__file__).resolve().parents[1] / 'yolopv2'
    demo_py = repo_dir / 'tools' / 'demo.py'
    if not demo_py.exists():
        print("ERROR: yolopv2/tools/demo.py not found.\n"
              "Did you add the submodule? -> git submodule add https://github.com/CAIC-AD/YOLOPv2 yolopv2")
        sys.exit(1)

    cmd = [
        sys.executable, str(demo_py),
        '--weights', str(Path(args.weights)),
        '--source', *[str(Path(p)) for p in args.inputs],
        '--img', str(args.imgw), str(args.imgh),
        '--save-dir', str(Path(args.outdir))
    ]
    print("Running:", " ".join(cmd))
    subprocess.check_call(cmd)
    print(f"Done. Check outputs in: {args.outdir}")

if __name__ == '__main__':
    main()
