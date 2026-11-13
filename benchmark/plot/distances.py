#! /usr/bin/env python3
"""Plot the timings of `coal-benchmark-distances` from its json output.

`benchmark/distances.cpp` measures, for each pair of shapes, the time of the
generic GJK implementation and the time of the specialization written for that
pair.  Every run is named `<shape1>_<shape2>_<implementation>`, where
`<implementation>` is either `gjk` or `specialized`:

    ./bin/coal-benchmark-distances --benchmark_out=benchmark.json

Three plots are made from that json, with the times converted to microseconds.
Each run measures one iteration, i.e. `num_transforms` collision pairs:

  1. a categorical heatmap of the GJK times,
  2. the same heatmap with the specialization, using GJK for the pairs
     that have no specialization (hatched cells),
  3. a stacked bar chart with both times of every collision pair.

To run this script:
    pixi run -e plot plot-distances benchmark.json --show
"""

import argparse
import json
import math
import statistics
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
# Time, in microseconds, and the implementation it was measured with, or None
# when the pair has no measurement.
Measurement = tuple[float, Implementation | None]

NUM_ITERATIONS = 1000
IMPLEMENTATIONS = ("gjk", "specialized")

# Time unit of the json output, converted to microseconds.
TIME_UNITS = {"ns": 1e-3, "us": 1.0, "ms": 1e3, "s": 1e6}

NO_MEASUREMENT_COLOR = "#d9d9d9"
BAR_COLORS = {"gjk": "#440154", "specialized": "#21918c"}
HATCH = "/"
NO_HATCH = "blank"


def read_benchmarks(path: Path, metric: str) -> dict[Pair, dict[Implementation, float]]:
    """Read the json output, keeping the order in which the runs appear in it.

    Returns `{(shape1, shape2): {implementation: time in microseconds}}`, where a
    time is NaN when the implementation is not available for the pair.
    """
    with path.open(encoding="utf-8") as stream:
        runs = json.load(stream).get("benchmarks", [])

    times: dict[Pair, dict[Implementation, float]] = defaultdict(dict)
    for run in runs:
        # The args of a benchmark, if any, are appended to its name.
        parts = run["name"].split("/")[0].split("_")
        implementation = parts[-1]
        if implementation not in IMPLEMENTATIONS or len(parts) != 3:
            print(f"warning: skipping {run['name']}", file=sys.stderr)
            continue

        pair = (parts[0], parts[1])
        unit = TIME_UNITS.get(run.get("time_unit", "ns"), 1e-3)
        times[pair][implementation] = (run[metric] * unit) / NUM_ITERATIONS

    # Only one of the two implementations may be available for a pair.
    measurements: dict[Pair, dict[str, float]] = {}
    for pair, measured in times.items():
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
    metric: str,
    measure: Callable[[Pair], Measurement],
    hatched: Implementation | None = "null",
) -> figure:
    """Make a heatmap with one cell per collision pair, colored by its time.

    `measure(pair)` returns the time to display and the implementation it comes
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
                "time": "-" if math.isnan(value) else f"{value:.1f}",
                "hatch": HATCH if implementation == hatched else NO_HATCH,
            }
        )

    measured = [row["value"] for row in rows if not math.isnan(row["value"])]
    mapper = LinearColorMapper(
        palette=Viridis256,
        # The implementation measured by the json may have no time at all, in which
        # case every cell is grey.
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
        ColorBar(color_mapper=mapper, title=f"{metric} (µs)", location="top"), "right"
    )
    tooltips = [
        ("collision pair", "@pair"),
        ("implementation", "@implementation"),
        ("time", "@time µs"),
    ]

    plot.add_tools(HoverTool(tooltips=tooltips))
    plot.xaxis.axis_label = "shape 2"
    plot.yaxis.axis_label = "shape 1"
    plot.grid.visible = False

    return plot


def stacked_bars(
    pairs: list[Pair],
    times: dict[Pair, dict[Implementation, float]],
    title: str,
    metric: str,
) -> figure:
    """Make a stacked bar chart with the GJK and specialization times of each pair.

    A pair that has no measurement for an implementation gets no segment for it.
    """
    data: dict[str, list[object]] = {
        "pair": [],
        IMPLEMENTATIONS[0]: [],
        IMPLEMENTATIONS[1]: [],
    }
    for pair in pairs:
        data["pair"].append("/".join(pair))
        for implementation in IMPLEMENTATIONS:
            value = times[pair][implementation]
            value = 0 if math.isnan(value) else value
            data[implementation].append(value)

    plot = figure(
        title=title,
        x_range=FactorRange(factors=["/".join(pair) for pair in pairs]),
        width=1100,
        height=400,
        tools=[],
        toolbar_location=None,
    )

    implementation_order = list(reversed(IMPLEMENTATIONS))
    plot.vbar_stack(
        implementation_order,
        x="pair",
        width=0.9,
        source=data,
        color=[BAR_COLORS[i] for i in implementation_order],
        legend_label=implementation_order,
    )
    tooltips = [
        ("collision pair", "@pair"),
        ("name", "$name"),
        ("time", "@$name µs"),
    ]

    plot.y_range.start = 0
    plot.x_range.range_padding = 0.1
    plot.xaxis.major_label_orientation = "vertical"
    plot.xaxis.axis_label = "collision pair"
    plot.yaxis.axis_label = f"{metric} (µs)"
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
        "--metric",
        choices=("real_time", "cpu_time"),
        default="real_time",
        help="time of the json to plot (default: real_time)",
    )
    parser.add_argument(
        "--show", action="store_true", help="show the plots instead of writing a file"
    )
    args = parser.parse_args()

    times = read_benchmarks(args.json, args.metric)
    if not times:
        print(f"error: no collision pair measured in {args.json}", file=sys.stderr)
        return 1

    pairs = list(times)
    shapes = shape_order(pairs)

    def gjk(pair: Pair) -> Measurement:
        value = times[pair]["gjk"]
        return (value, "gjk" if not math.isnan(value) else None)

    def specialized_or_gjk(pair: Pair) -> Measurement:
        for implementation in ("specialized", "gjk"):
            value = times[pair][implementation]
            if not math.isnan(value):
                return (value, implementation)
        return (math.nan, None)

    plots = [
        categorical_heatmap(
            pairs,
            shapes,
            "GJK, the generic implementation (grey: not managed)",
            args.metric,
            gjk,
        ),
        categorical_heatmap(
            pairs,
            shapes,
            "Specialization, hatched when there is no specialization",
            args.metric,
            specialized_or_gjk,
            hatched="gjk",
        ),
        stacked_bars(
            pairs, times, "Both implementations per collision pair", args.metric
        ),
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
