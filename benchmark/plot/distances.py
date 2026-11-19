#! /usr/bin/env python3
"""Plot the throughputs of `coal-benchmark-distances` from its json output.

`benchmark/distances.cpp` measures, for each pair of shapes, the throughput of
the generic GJK implementation and the throughput of the specialization written
for that pair.  Every run is named `<shape1>_<shape2>_<implementation>`, where
`<implementation>` is either `gjk` or `specialized`:

    ./bin/coal-benchmark-distances --benchmark_out=benchmark.json

Three plots are made from that json, plotting the number of collision pairs
computed per second.  Each run measures one iteration, i.e. `num_transforms`
collision pairs, and reports the rate of an iteration in its
`items_per_second` counter, counted against the real time of the run:

  1. a categorical heatmap of the GJK throughputs,
  2. the same heatmap with the specialization, using GJK for the pairs
     that have no specialization (hatched cells),
  3. a bar chart with both throughputs of every collision pair, the
     implementations nested in the pair.

To run this script:
    pixi run -e plot plot-distances benchmark.json --show
"""

import argparse
import json
import math
import sys
from collections import defaultdict
from collections.abc import Callable
from pathlib import Path

from bokeh.core.properties import field
from bokeh.io import output_file, save, show
from bokeh.layouts import column
from bokeh.models import (
    ColorBar,
    ColumnDataSource,
    FactorRange,
    HoverTool,
    LinearColorMapper,
)
from bokeh.palettes import Viridis256
from bokeh.plotting import figure
from bokeh.transform import transform

Pair = tuple[str, str]
Implementation = str
# Throughput, in millions of collision pairs per second, and the implementation
# it was measured with, or None when the pair has no measurement.
Measurement = tuple[float, Implementation | None]

IMPLEMENTATIONS = ("gjk", "specialized")

# `items_per_second` counter of the json, converted to millions of collision
# pairs per second, and the unit the axes and the color bar are labeled with.
RATE_SCALE = 1e-6
RATE_UNIT = "M collision pairs/s"

NO_MEASUREMENT_COLOR = "#d9d9d9"
BAR_COLORS = {"gjk": "#440154", "specialized": "#21918c"}
HATCH = "/"
NO_HATCH = "blank"


def format_rate(rate: float) -> str:
    """Rate in millions of collision pairs per second, for the tooltips."""
    return f"{rate:.3g}"


def read_rates(path: Path) -> dict[Pair, dict[Implementation, float]]:
    """Read the json output, keeping the order in which the runs appear in it.

    Returns `{(shape1, shape2): {implementation: rate in millions of collision
    pairs per second}}`, where a rate is NaN when the implementation is not
    available for the pair.  `SetItemsProcessed` counts one item per collision
    pair, so `items_per_second` is the throughput of the pair.
    """
    with path.open(encoding="utf-8") as stream:
        runs = json.load(stream).get("benchmarks", [])

    rates: dict[Pair, dict[Implementation, float]] = defaultdict(dict)
    for run in runs:
        # The args of a benchmark, if any, are appended to its name.
        parts = run["name"].split("/")[0].split("_")
        implementation = parts[-1]
        if implementation not in IMPLEMENTATIONS or len(parts) != 3:
            print(f"warning: skipping {run['name']}", file=sys.stderr)
            continue
        if "items_per_second" not in run:
            print(
                f"warning: no items_per_second counter for {run['name']}",
                file=sys.stderr,
            )
            continue

        pair = (parts[0], parts[1])
        rates[pair][implementation] = run["items_per_second"] * RATE_SCALE

    # Only one of the two implementations may be available for a pair.
    measurements: dict[Pair, dict[str, float]] = {}
    for pair, measured in rates.items():
        measurements[pair] = {}
        for implementation in IMPLEMENTATIONS:
            values = measured.get(implementation, math.nan)
            measurements[pair][implementation] = values

    return measurements


def shape_order(pairs: list[Pair]) -> list[str]:
    """Shapes in the order they appear in the json, as rows of the heatmap."""
    shapes = []
    for pair in pairs:
        for shape in pair:
            if shape not in shapes:
                shapes.append(shape)
    return shapes


def as_source(rows: list[dict[str, object]]) -> ColumnDataSource:
    """Turn a list of rows, all having the same keys, into a data source."""
    return ColumnDataSource({key: [row[key] for row in rows] for key in rows[0]})


def categorical_heatmap(
    pairs: list[Pair],
    shapes: list[str],
    title: str,
    measure: Callable[[Pair], Measurement],
    hatched: Implementation | None = "null",
) -> figure:
    """Make a heatmap with one cell per collision pair, colored by its rate.

    `measure(pair)` returns the rate to display and the implementation it comes
    from, or None when the pair has no measurement, which leaves an empty cell.
    `hatched` is the implementation whose cells get a hatch pattern.
    """
    rows = []
    for pair in pairs:
        value, implementation = measure(pair)
        rows.append(
            {
                "pair": "_".join(pair),
                "shape1": pair[0],
                "shape2": pair[1],
                "value": value,
                "implementation": implementation or "-",
                "rate": "-" if math.isnan(value) else format_rate(value),
                "hatch": HATCH if implementation == hatched else NO_HATCH,
            }
        )

    measured = [row["value"] for row in rows if not math.isnan(row["value"])]
    mapper = LinearColorMapper(
        palette=Viridis256,
        # The implementation measured by the json may have no rate at all, in
        # which case every cell is grey.
        low=min(measured) if measured else 0.0,
        high=max(measured) if measured else 1.0,
        nan_color=NO_MEASUREMENT_COLOR,
    )

    plot = figure(
        title=title,
        x_range=FactorRange(factors=shapes),
        # The first shape of a pair is a row, so read the matrix top down.
        y_range=FactorRange(factors=list(reversed(shapes))),
        width=620,
        height=420,
        tools=[],
        toolbar_location=None,
    )
    plot.rect(
        x="shape2",
        y="shape1",
        width=1,
        height=1,
        source=as_source(rows),
        fill_color=transform("value", mapper),
        hatch_pattern=field("hatch"),
        line_color=None,
    )
    plot.add_layout(
        ColorBar(color_mapper=mapper, title=RATE_UNIT, location="top"), "right"
    )
    tooltips = [
        ("collision pair", "@pair"),
        ("implementation", "@implementation"),
        ("rate", f"@rate {RATE_UNIT}"),
    ]

    plot.add_tools(HoverTool(tooltips=tooltips))
    plot.xaxis.axis_label = "shape 2"
    plot.yaxis.axis_label = "shape 1"
    plot.grid.visible = False

    return plot


def nested_bars(
    pairs: list[Pair],
    rates: dict[Pair, dict[Implementation, float]],
    title: str,
) -> figure:
    """Make a bar chart with the GJK and specialization rates of each pair."""
    rows: list[dict[str, object]] = []
    factors: list[tuple[str, str]] = []
    for pair in pairs:
        name = "/".join(pair)
        for implementation in IMPLEMENTATIONS:
            value = rates[pair][implementation]
            if math.isnan(value):
                continue
            factors.append((name, implementation))
            rows.append(
                {
                    "factor": (name, implementation),
                    "pair": name,
                    "implementation": implementation,
                    "rate": value,
                    "rate_text": format_rate(value),
                    "color": BAR_COLORS[implementation],
                }
            )

    plot = figure(
        title=title,
        x_range=FactorRange(factors=factors),
        width=1100,
        height=400,
        tools=["xwheel_zoom", "xpan", "reset"],
        active_scroll="xwheel_zoom",
        active_drag="xpan",
        toolbar_location="above",
    )
    plot.vbar(
        x="factor",
        top="rate",
        width=0.9,
        source=as_source(rows),
        fill_color=field("color"),
        legend_field="implementation",
        line_color=None,
    )

    plot.y_range.start = 0
    plot.x_range.range_padding = 0.1
    plot.xaxis.major_label_text_alpha = 0
    plot.xaxis.group_label_orientation = "vertical"
    plot.xaxis.axis_label = "collision pair"
    plot.yaxis.axis_label = RATE_UNIT
    plot.xgrid.grid_line_color = None
    tooltips = [
        ("collision pair", "@pair"),
        ("implementation", "@implementation"),
        ("rate", f"@rate_text {RATE_UNIT}"),
    ]
    plot.add_tools(HoverTool(tooltips=tooltips))
    plot.legend.location = "top_left"
    plot.legend.orientation = "horizontal"

    return plot


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "json", type=Path, help="json output of the coal-benchmark-distances benchmark"
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="html file to write (default: the json path, with the .html suffix)",
    )
    parser.add_argument(
        "--show", action="store_true", help="show the plots instead of writing a file"
    )
    args = parser.parse_args()

    rates = read_rates(args.json)
    if not rates:
        print(f"error: no collision pair measured in {args.json}", file=sys.stderr)
        return 1

    pairs = list(rates)
    shapes = shape_order(pairs)

    def gjk(pair: Pair) -> Measurement:
        value = rates[pair]["gjk"]
        return (value, "gjk" if not math.isnan(value) else None)

    def specialized_or_gjk(pair: Pair) -> Measurement:
        for implementation in ("specialized", "gjk"):
            value = rates[pair][implementation]
            if not math.isnan(value):
                return (value, implementation)
        return (math.nan, None)

    plots = [
        categorical_heatmap(
            pairs,
            shapes,
            "GJK, the generic implementation (grey: not managed)",
            gjk,
        ),
        categorical_heatmap(
            pairs,
            shapes,
            "Specialization, hatched when there is no specialization",
            specialized_or_gjk,
            hatched="gjk",
        ),
        nested_bars(pairs, rates, "Both implementations per collision pair"),
    ]

    output_file(
        args.output or args.json.with_suffix(".html"), title="coal-benchmark-distances"
    )
    if args.show:
        show(column(*plots))
    else:
        save(column(*plots))

    return 0


if __name__ == "__main__":
    sys.exit(main())
