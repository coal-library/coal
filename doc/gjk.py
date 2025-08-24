#!/usr/bin/env python3
import pdb
import sys

# ABC = AB^AC
# (ABC^AJ).a = (j.c - j.b) a.a + (j.a - j.c) b.a + (j.b - j.a) c.a, for j = b or c

segment_fmt = "{j}a_aa"
plane_fmt = ""
edge_fmt = "{j}a * {b}a_{c}a + {j}{b} * {c}a_aa - {j}{c} * {b}a_aa"

# These checks must be negative and not positive, as in the cheat sheet.
# They are the same as in the cheat sheet, except that we consider (...).dot(A)
# instead of (...).dot(-A)
plane_tests = ["C.dot (a_cross_b)", "D.dot(a_cross_c)", "-D.dot(a_cross_b)"]
checks = (
    plane_tests
    + [edge_fmt.format(**{"j": j, "b": "b", "c": "c"}) for j in ["b", "c"]]
    + [edge_fmt.format(**{"j": j, "b": "c", "c": "d"}) for j in ["c", "d"]]
    + [edge_fmt.format(**{"j": j, "b": "d", "c": "b"}) for j in ["d", "b"]]
    + [segment_fmt.format(**{"j": j}) for j in ["b", "c", "d"]]
)
checks_hr = (
    ["ABC.AO >= 0", "ACD.AO >= 0", "ADB.AO >= 0"]
    + ["(ABC ^ {}).AO >= 0".format(n) for n in ["AB", "AC"]]
    + ["(ACD ^ {}).AO >= 0".format(n) for n in ["AC", "AD"]]
    + ["(ADB ^ {}).AO >= 0".format(n) for n in ["AD", "AB"]]
    + ["AB.AO >= 0", "AC.AO >= 0", "AD.AO >= 0"]
)

# weights of the checks.
weights = (
    [
        2,
    ]
    * 3
    + [
        3,
    ]
    * 6
    + [
        1,
    ]
    * 3
)

# Segment tests first, because they have lower weight.
# tests = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, ]
tests = [
    9,
    10,
    11,
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
]
assert len(tests) == len(checks)
assert sorted(tests) == list(range(len(tests)))

regions = [
    "ABC",
    "ACD",
    "ADB",
    "AB",
    "AC",
    "AD",
    "A",
    "Inside",
]
cases = list(range(len(regions)))

# The following 3 lists refer to table doc/GJK_tetrahedra_boolean_table.ods

# A check ID is (+/- (index+1)) where a minus sign encodes a NOT operation
# and index refers to an index in list checks.

# definitions is a list of list of check IDs to be ANDed.
# For instance, a0.a3.!a4 -> [ 1, 4, -5]
definitions = [
    [1, 4, -5],
    [2, 6, -7],
    [3, 8, -9],
    [-4, 9, 10],
    [-6, 5, 11],
    [-8, 7, 12],
    [-10, -11, -12],
    [-1, -2, -3],
]
# conditions is a list of (list of (list of check IDs to be ANDed) to be ORed).
conditions = [
    [],
    [],
    [],
    [],
    [],
    [],
    [],
    [],  # [ [10, 11, 12], ], # I don't think this is always true...
]
# rejections is a list of (list of (list of check IDs to be ANDed) to be ORed).
rejections = [
    [
        [2, 6, 7],
        [3, -8, -9],
    ],
    [
        [3, 8, 9],
        [1, -4, -5],
    ],
    [
        [1, 4, 5],
        [2, -6, -7],
    ],
    [
        [-1, -3],
    ],
    [
        [-2, -1],
    ],
    [
        [-3, -2],
    ],
    [
        [4, -5],
        [6, -7],
        [8, -9],
    ],
    [],
]

implications = [
    [
        [
            4,
            5,
            10,
        ],
        [11],
    ],
    [
        [
            6,
            7,
            11,
        ],
        [12],
    ],
    [
        [
            8,
            9,
            12,
        ],
        [10],
    ],
    [
        [
            -4,
            -5,
            11,
        ],
        [10],
    ],
    [
        [
            -6,
            -7,
            12,
        ],
        [11],
    ],
    [
        [
            -8,
            -9,
            10,
        ],
        [12],
    ],
    [[1, 4, 5, 6], [-7]],
    [[2, 6, 9, 8], [-9]],
    [[3, 8, 9, 4], [-5]],
    [
        [
            -4,
            5,
            10,
        ],
        [-11],
    ],
    [
        [
            4,
            -5,
            -10,
        ],
        [11],
    ],
    [
        [
            -6,
            7,
            11,
        ],
        [-12],
    ],
    [
        [
            6,
            -7,
            -11,
        ],
        [12],
    ],
    [
        [
            -8,
            9,
            12,
        ],
        [-10],
    ],
    [
        [
            8,
            -9,
            -12,
        ],
        [10],
    ],
    [[10, 3, 9, -12, 4, -5], [1]],
    [[10, -3, 1, -4], [9]],
    [[10, -3, -1, 2, -6, 11], [5]],
    [[-10, 11, 2, -12, -5, -1], [6]],
    [[-10, 11, -2, 1, 5], [-6]],
    [[-10, -11, 12, 1, -7, -2, 4], [-5]],
    [[-10, -11, 12, -3, 2, 7], [-8]],
    [[-10, -11, 12, -3, -2], [-1]],
]


def set_test_values(current_tests, test_values, itest, value):
    def satisfies(values, indices):
        for k in indices:
            if k > 0 and values[k - 1] is not True:
                return False
            if k < 0 and values[-k - 1] is not False:
                return False
        return True

    remaining_tests = list(current_tests)
    next_test_values = list(test_values)

    remaining_tests.remove(itest)
    next_test_values[itest] = value
    rerun = True
    while rerun:
        rerun = False
        for impl in implications:
            if satisfies(next_test_values, impl[0]):
                for id in impl[1]:
                    k = (id - 1) if id > 0 else (-id - 1)
                    if k in remaining_tests:
                        next_test_values[k] = id > 0
                        remaining_tests.remove(k)
                        rerun = True
                    else:
                        if next_test_values[k] != (id > 0):
                            raise ValueError("Absurd case")
    return remaining_tests, next_test_values


def set_tests_values(current_tests, test_values, itests, values):
    for itest, value in zip(itests, values):
        current_tests, test_values = set_test_values(
            current_tests, test_values, itest, value
        )
    return current_tests, test_values


def apply_test_values(cases, test_values):
    def canSatisfy(values, indices):
        for k in indices:
            if k > 0 and values[k - 1] is False:
                return False
            if k < 0 and values[-k - 1] is True:
                return False
        return True

    def satisfies(values, indices):
        for k in indices:
            if k > 0 and values[k - 1] is not True:
                return False
            if k < 0 and values[-k - 1] is not False:
                return False
        return True

    # Check all cases.
    left_cases = []
    for case in cases:
        defi = definitions[case]
        conds = conditions[case]
        rejs = rejections[case]
        if satisfies(test_values, defi):
            # A definition is True, stop recursion
            return [case]
        if not canSatisfy(test_values, defi):
            continue
        for cond in conds:
            if satisfies(test_values, cond):
                # A condition is True, stop recursion
                return [case]
        append = True
        for rej in rejs:
            if satisfies(test_values, rej):
                # A rejection is True, discard this case
                append = False
                break
        if append:
            left_cases.append(case)
    return left_cases


def max_number_of_tests(
    current_tests,
    cases,
    test_values=[
        None,
    ]
    * len(tests),
    prevBestScore=float("inf"),
    prevScore=0,
):
    for test in current_tests:
        assert test_values[test] is None, "Test " + str(test) + " already performed"

    left_cases = apply_test_values(cases, test_values)

    if len(left_cases) == 1:
        return prevScore, {
            "case": left_cases[0],
        }
    elif len(left_cases) == 0:
        return prevScore, {
            "case": None,
            "comments": [
                "applied " + str(test_values),
                "to " + ", ".join([regions[c] for c in cases]),
            ],
        }

    assert len(current_tests) > 0, "No more test but " + str(left_cases) + " remains"

    currentBestScore = prevBestScore
    bestScore = float("inf")
    bestOrder = [None, None]
    for i, test in enumerate(current_tests):
        assert bestScore >= currentBestScore

        currentScore = prevScore + len(left_cases) * weights[test]
        # currentScore = prevScore + weights[test]
        if currentScore > currentBestScore:  # Cannot do better -> stop
            continue

        try:
            remaining_tests, next_test_values = set_test_values(
                current_tests, test_values, test, True
            )
        except ValueError:
            remaining_tests = None

        if remaining_tests is not None:
            # Do not put this in try catch as I do not want other ValueError to be
            # understood as an infeasible branch.
            score_if_t, order_if_t = max_number_of_tests(
                remaining_tests,
                left_cases,
                next_test_values,
                currentBestScore,
                currentScore,
            )
            if score_if_t >= currentBestScore:  # True didn't do better -> stop
                continue
        else:
            score_if_t, order_if_t = prevScore, None

        try:
            remaining_tests, next_test_values = set_test_values(
                current_tests, test_values, test, False
            )
        except ValueError:
            remaining_tests = None

        if remaining_tests is not None:
            # Do not put this in try catch as I do not want other ValueError to be
            # understood as an infeasible branch.
            score_if_f, order_if_f = max_number_of_tests(
                remaining_tests,
                left_cases,
                next_test_values,
                currentBestScore,
                currentScore,
            )
        else:
            score_if_f, order_if_f = prevScore, None

        currentScore = max(score_if_t, score_if_f)
        if currentScore < bestScore:
            if currentScore < currentBestScore:
                bestScore = currentScore
                bestOrder = {"test": test, "true": order_if_t, "false": order_if_f}
                # pdb.set_trace()
                currentBestScore = currentScore
                if len(tests) == len(current_tests):
                    print("New best score: {}".format(currentBestScore))

    return bestScore, bestOrder


def printComments(order, indent, file):
    if "comments" in order:
        for comment in order["comments"]:
            print(indent + "// " + comment, file=file)


def printOrder(order, indent="", start=True, file=sys.stdout, curTests=[]):
    if start:
        print(
            "bool GJK::projectTetrahedraOrigin(const Simplex& current, Simplex& next)",
            file=file,
        )
        print("{", file=file)
        print(
            indent + "// The code of this function was generated using doc/gjk.py",
            file=file,
        )
        print(indent + "const vertex_id_t a = 3, b = 2, c = 1, d = 0;", file=file)
        for v in "abcd":
            print(
                indent
                + "const Vec3s& {} (current.vertex[{}]->w);".format(v.upper(), v),
                file=file,
            )
        print(indent + "const Scalar aa = A.squaredNorm();".format(), file=file)
        for v in "dcb":
            for m in "abcd":
                if m <= v:
                    print(
                        indent
                        + "const Scalar {0}{1}    = {2}.dot({3});".format(
                            v, m, v.upper(), m.upper()
                        ),
                        file=file,
                    )
                else:
                    print(
                        indent + "const Scalar& {0}{1}    = {1}{0};".format(v, m),
                        file=file,
                    )
            print(indent + "const Scalar {0}a_aa = {0}a - aa;".format(v), file=file)
        for l0, l1 in zip("bcd", "cdb"):
            print(
                indent + "const Scalar {0}a_{1}a = {0}a - {1}a;".format(l0, l1),
                file=file,
            )
        for v in "bc":
            print(
                indent + "const Vec3s a_cross_{0} = A.cross({1});".format(v, v.upper()),
                file=file,
            )
        print("", file=file)
        print("#define REGION_INSIDE()               " + indent + "\\", file=file)
        print(indent + "  ray.setZero();                      \\", file=file)
        print(indent + "  next.vertex[0] = current.vertex[d]; \\", file=file)
        print(indent + "  next.vertex[1] = current.vertex[c]; \\", file=file)
        print(indent + "  next.vertex[2] = current.vertex[b]; \\", file=file)
        print(indent + "  next.vertex[3] = current.vertex[a]; \\", file=file)
        print(indent + "  next.rank=4;                        \\", file=file)
        print(indent + "  return true;", file=file)
        print("", file=file)

    if "case" in order:
        case = order["case"]
        if case is None:
            print(
                indent + "// There are no case corresponding to this set of tests.",
                file=file,
            )
            printComments(order, indent, file)
            print(indent + "assert(false);", file=file)
            return
        region = regions[case]
        print(indent + "// Region " + region, file=file)
        printComments(order, indent, file)
        toFree = ["b", "c", "d"]
        if region == "Inside":
            print(indent + "REGION_INSIDE()", file=file)
            toFree = []
        elif region == "A":
            print(indent + "originToPoint (current, a, A, next, ray);", file=file)
        elif len(region) == 2:
            region[0]
            B = region[1]
            print(
                indent
                + "originToSegment "
                "(current, a, {b}, A, {B}, {B}-A, -{b}a_aa, next, ray);".format(
                    **{"b": B.lower(), "B": B}
                ),
                file=file,
            )
            toFree.remove(B.lower())
        elif len(region) == 3:
            B = region[1]
            C = region[2]
            test = plane_tests[["ABC", "ACD", "ADB"].index(region)]
            if test.startswith("-"):
                test = test[1:]
            else:
                test = "-" + test
            print(
                indent
                + "originToTriangle "
                "(current, a, {b}, {c}, ({B}-A).cross({C}-A), {t}, next, ray);".format(
                    **{"b": B.lower(), "c": C.lower(), "B": B, "C": C, "t": test}
                ),
                file=file,
            )
            toFree.remove(B.lower())
            toFree.remove(C.lower())
        else:
            assert False, "Unknown region " + region
        for pt in toFree:
            print(
                indent + "free_v[nfree++] = current.vertex[{}];".format(pt), file=file
            )
    else:
        assert "test" in order and "true" in order and "false" in order
        check = checks[order["test"]]
        check_hr = checks_hr[order["test"]]
        printComments(order, indent, file)
        nextTests_t = curTests + [
            "a" + str(order["test"] + 1),
        ]
        nextTests_f = curTests + [
            "!a" + str(order["test"] + 1),
        ]
        if order["true"] is None:
            if order["false"] is None:
                print(
                    indent
                    + """assert(false && "Case {} should never happen.");""".format(
                        check_hr
                    )
                )
            else:
                print(
                    indent
                    + "assert(!({} <= 0)); // Not {} / {}".format(
                        check, check_hr, ".".join(nextTests_f)
                    ),
                    file=file,
                )
                printOrder(
                    order["false"],
                    indent=indent,
                    start=False,
                    file=file,
                    curTests=nextTests_f,
                )
        elif order["false"] is None:
            print(
                indent
                + "assert({} <= 0); // {} / {}".format(
                    check, check_hr, ".".join(nextTests_t)
                ),
                file=file,
            )
            printOrder(
                order["true"],
                indent=indent,
                start=False,
                file=file,
                curTests=nextTests_t,
            )
        else:
            print(
                indent
                + "if ({} <= 0) {{ // if {} / {}".format(
                    check, check_hr, ".".join(nextTests_t)
                ),
                file=file,
            )
            printOrder(
                order["true"],
                indent=indent + "  ",
                start=False,
                file=file,
                curTests=nextTests_t,
            )
            print(
                indent
                + "}} else {{ // not {} / {}".format(check_hr, ".".join(nextTests_f)),
                file=file,
            )
            printOrder(
                order["false"],
                indent=indent + "  ",
                start=False,
                file=file,
                curTests=nextTests_f,
            )
            print(indent + "}} // end of {}".format(check_hr), file=file)

    if start:
        print("", file=file)
        print("#undef REGION_INSIDE", file=file)
        print(indent + "return false;", file=file)
        print("}", file=file)


def unit_tests():
    # a4, a5, a10, a11, a12
    cases = list(range(len(regions)))
    pdb.set_trace()
    left_cases = apply_test_values(
        cases,
        test_values=[
            None,
            None,
            None,
            True,
            True,
            None,
            None,
            None,
            None,
            True,
            True,
            True,
        ],
    )
    assert len(left_cases) > 1


# unit_tests()

score, order = max_number_of_tests(tests, cases)

print(score)
printOrder(order, indent="  ")

# TODO add weights such that:
# - it is preferred to have all the use of one check in one branch.
#   idea: ponderate by the number of remaining tests.
