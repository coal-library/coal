class XmlDocString(object):
    def __init__(self, index):
        self.index = index
        self.tags = {
            "para": self.para,
            "ref": self.ref,
            "briefdescription": self.otherTags,
            "detaileddescription": self.otherTags,
            "parameterlist": self.parameterlist,
            "parameterdescription": self.otherTags,
            "emphasis": self.emphasis,
            "simplesect": self.simplesect,
            "formula": self.formula,
            "itemizedlist": self.itemizedlist,
            "listitem": self.listitem,
        }
        self.unkwownTags = set()
        self.unkwownReferences = dict()
        self._linesep = '\\n"\n"'

        try:
            from pylatexenc.latex2text import LatexNodes2Text

            self.latex = LatexNodes2Text()
        except ImportError:
            self.latex = None

    def clear(self):
        self.lines = []
        self.unkwownTags.clear()
        self.unkwownReferences.clear()

    def writeErrors(self, output):
        ret = False
        for t in self.unkwownTags:
            output.warn("Unknown tag: ", t)
            ret = True
        for ref, node in self.unkwownReferences.items():
            output.warn("Unknown reference: ", ref, node.text)
            ret = True
        return ret

    def _write(self, str):
        nlines = str.split("\n")
        if len(self.lines) == 0:
            self.lines += nlines
        else:
            self.lines[-1] += nlines[0]
            self.lines += nlines[1:]
            # self.lines += nlines[1:]

    def _newline(self, n=1):
        self.lines.extend(
            [
                "",
            ]
            * n
        )

    def _clean(self):
        s = 0
        for line in self.lines:
            if len(line.strip()) == 0:
                s += 1
            else:
                break
        e = len(self.lines)
        for line in reversed(self.lines):
            if len(line.strip()) == 0:
                e -= 1
            else:
                break
        self.lines = self.lines[s:e]

    def getDocString(self, brief, detailled, output):
        self.clear()
        if brief is not None:
            self.visit(brief)
        if detailled is not None and len(detailled.getchildren()) > 0:
            if brief is not None:
                self._newline()
            self.visit(detailled)
        from sys import version_info

        self.writeErrors(output)
        self._clean()
        if version_info[0] == 2:
            return self._linesep.join(self.lines).encode("utf-8")
        else:
            return self._linesep.join(self.lines)

    def visit(self, node):
        assert isinstance(node.tag, str)
        tag = node.tag
        if tag not in self.tags:
            self.unknownTag(node)
        else:
            self.tags[tag](node)

    def unknownTag(self, node):
        self.unkwownTags.add(node.tag)
        self.otherTags(node)

    def otherTags(self, node):
        if node.text:
            self._write(node.text.strip().replace('"', r"\""))
        for c in node.iterchildren():
            self.visit(c)
            if c.tail:
                self._write(c.tail.strip().replace('"', r"\""))

    def emphasis(self, node):
        self._write("*")
        self.otherTags(node)
        self._write("*")

    def simplesect(self, node):
        self._write(node.attrib["kind"].title() + ": ")
        self.otherTags(node)

    def para(self, node):
        if node.text:
            self._write(node.text.replace('"', r"\""))
        for c in node.iterchildren():
            self.visit(c)
            if c.tail:
                self._write(c.tail.replace('"', r"\""))
        self._newline()

    def ref(self, node):
        refid = node.attrib["refid"]
        if self.index.hasref(refid):
            self._write(self.index.getref(refid).name)
        else:
            self.unkwownReferences[refid] = node
            self._write(node.text)
        assert len(node.getchildren()) == 0

    def parameterlist(self, node):
        self._newline()
        self._write(node.attrib["kind"].title())
        self._newline()
        for item in node.iterchildren("parameteritem"):
            self.parameteritem(item)

    def parameteritem(self, node):
        indent = "  "
        self._write(indent + "- ")
        # should contain two children
        assert len(node.getchildren()) == 2
        namelist = node.find("parameternamelist")
        desc = node.find("parameterdescription")
        sep = ""
        for name in namelist.iterchildren("parametername"):
            self._write(sep + name.text)
            sep = ", "
        self._write(" ")
        self.visit(desc)

    def itemizedlist(self, node):
        self._newline()
        self.otherTags(node)

    def listitem(self, node):
        self._write("- ")
        self.otherTags(node)

    def formula(self, node):
        if node.text:
            if self.latex is None:
                self._write(node.text.strip())
            else:
                self._write(self.latex.latex_to_text(node.text))
