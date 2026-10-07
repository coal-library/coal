import matplotlib.pyplot as plt
import numpy as np

interactive = False

m = 1.0
b = 1.2

mb = m + b

X = np.array([-mb / 2, 0, m, mb, 2 * mb])
# X = np.linspace(-1, 4., 21)


def dlb(d):
    if d < 0:
        return None
    if d > mb:
        u = d - mb
        return mb - m + u / 2
    return d - m


plt.figure(figsize=(9, 3.5))
# plt.plot(X, X-m, ":k")
# plt.plot([m+b, X[-1]], [b, b], ":k")
plt.fill_between(
    [m + b, X[-1]],
    [b, b],
    [b, X[-1] - m],
    alpha=0.2,
    hatch="|",
    facecolor="g",
    label="Distance lower band area",
)
plt.plot(X, [dlb(x) for x in X], "-g", label="distance lower bound")
# plt.plot([X[0], m, m, X[-1]], [0, 0, b, b], ":k")
plt.axvspan(X[0], m, alpha=0.5, hatch="\\", facecolor="r", label="Collision area")


ax = plt.gca()
ax.set_xlabel("Object distance")
ax.set_xticks([0, m, mb])
ax.set_xticklabels(["0", "security margin", "security margin\n+ break distance"])
ax.set_yticks([0, b])
ax.set_yticklabels(["0", "break distance"])
ax.grid(which="major", ls="solid")
ax.grid(which="minor", ls="dashed")

plt.axvline(0, ls="solid")
# plt.axvline(m, ls="dashed", label="margin")
# plt.axvline(mb, ls="dashed")
plt.axhline(0.0, ls="solid")

plt.title("Collision and distance lower band")
plt.legend(loc="lower right")
if interactive:
    plt.show()
else:
    from os import path

    dir_path = path.dirname(path.realpath(__file__))
    plt.savefig(
        path.join(dir_path, "distance_computation.png"),
        bbox_inches="tight",
        orientation="landscape",
    )
