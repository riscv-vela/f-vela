import os
import sys

# Import the sibling module regardless of the current working directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from profile_graph import *  # noqa: E402,F401,F403

if len(sys.argv) < 2:
    print("Usage: python profile.py <logfile> [output.png]")
    sys.exit(1)

log_path = sys.argv[1]

result = read_log(log_path)

visualize(result)

# Always save a PNG next to the log (works headless), then try to show a window.
out_png = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(log_path)[0] + ".png"
plt.savefig(out_png, dpi=150, bbox_inches="tight")
print("Saved:", out_png)

try:
    plt.show()
except Exception as e:  # no display available, etc.
    print("(skipping interactive show: %s)" % e)
