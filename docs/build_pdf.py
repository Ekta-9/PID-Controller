"""
build_pdf.py - Turn docs/pid_guide.html into PID_Control_Guide.pdf using Chrome or Edge
in "headless" mode (no window). Run the figure scripts first:  python run_all.py
Run:  python docs/build_pdf.py
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
HTML = os.path.join(HERE, "pid_guide.html")
OUT = os.path.join(os.path.dirname(HERE), "PID_Control_Guide.pdf")

CANDIDATES = [
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    shutil.which("google-chrome") or "", shutil.which("chromium") or "", shutil.which("msedge") or "",
]
browser = next((c for c in CANDIDATES if c and os.path.exists(c)), None)
if browser is None:
    sys.exit("Chrome or Edge not found. Alternatively open docs/pid_guide.html and use Print -> Save as PDF.")

subprocess.run([browser, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                f"--print-to-pdf={OUT}", "file:///" + HTML.replace("\\", "/")], check=True)
print("saved", OUT)
