import csv
import sys

import matplotlib.pyplot as plt

filename = sys.argv[1]

with open(filename, "r") as file:
    reader = csv.reader(file, strict=True)
    fieldnames = None
    # fieldnames = reader.fieldnames
    converters = (str, int, int, int, float, lambda x: float(x) * 1e-3)

    for row in reader:
        if fieldnames is None:
            fieldnames = [n.strip() for n in row]
            values = []
            continue

        values.append([c(v) for v, c in zip(row, converters)])

request1 = values[: int(len(values) / 2)]
request2 = values[int(len(values) / 2) :]

Ntransforms = 1
while values[0][0:3] == values[Ntransforms][0:3]:
    Ntransforms += 1

splitMethods = ["avg", "med", "cen"]
type = [
    "o",
    "or",
    "r",
]
BVs = sorted({v[0] for v in request1[::Ntransforms]})
xvals = [
    BVs.index(v[0]) + len(BVs) * v[2] + 3 * len(BVs) * v[1]
    for v in request1[::Ntransforms]
]
cases = [
    v[0] + " " + type[v[1]] + " " + splitMethods[v[2]] for v in request1[::Ntransforms]
]

idx_reorder = sorted(range(len(xvals)), key=lambda i: xvals[i])


def reorder(v):
    return [v[i] for i in idx_reorder]


xvals_s = reorder(xvals)
cases_s = reorder(cases)

onlyLB = True
# Time
plt.figure(0)
for i in range(Ntransforms):
    if not onlyLB:
        plt.plot(
            xvals_s,
            reorder([v[5] for v in request1[i::Ntransforms]]),
            "-.o",
            label=str(i),
        )
    plt.plot(
        xvals_s,
        reorder([v[5] for v in request2[i::Ntransforms]]),
        ":+",
        label=str(i) + "+lb",
    )

plt.legend()
plt.xticks(ticks=xvals_s, labels=cases_s, rotation=90)
plt.ylabel("Time (us)")
plt.yscale("log")

# Time
plt.figure(2)
for k in range(0, len(request1), Ntransforms):
    if not onlyLB:
        plt.plot(
            [
                xvals[int(k / Ntransforms)],
            ],
            sum([v[5] for v in request1[k : k + Ntransforms]]) / Ntransforms,
        )
    plt.plot(
        [
            xvals[int(k / Ntransforms)],
        ],
        sum([v[5] for v in request2[k : k + Ntransforms]]) / Ntransforms,
    )

plt.plot(
    xvals_s,
    reorder(
        [
            sum([v[5] for v in request2[k : k + Ntransforms]]) / Ntransforms
            for k in range(0, len(request1), Ntransforms)
        ]
    ),
)

plt.xticks(ticks=xvals_s, labels=cases_s, rotation=90)
plt.ylabel("Time (us)")
plt.yscale("log")

# Distance lower bound
plt.figure(1)
for i in range(Ntransforms):
    if request2[i][3] > 0:
        continue
    plt.plot(
        xvals_s,
        reorder([v[4] for v in request2[i::Ntransforms]]),
        ":+",
        label=str(i) + "+lb",
    )

plt.legend()
plt.ylabel("Distance lower bound")
plt.xticks(ticks=xvals_s, labels=cases_s, rotation=90)

plt.show()
