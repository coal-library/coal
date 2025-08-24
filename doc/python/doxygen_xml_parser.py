#!/usr/bin/python3
# ruff: noqa: E501

from __future__ import print_function
from lxml import etree
from os import path
from xml_docstring import XmlDocString
import sys

template_file_header = """#ifndef DOXYGEN_AUTODOC_{header_guard}
#define DOXYGEN_AUTODOC_{header_guard}

#include "{path}/doxygen.hh"
"""
template_file_footer = """
#endif // DOXYGEN_AUTODOC_{header_guard}
"""

template_class_doc = """
template <{tplargs}>
struct class_doc_impl< {classname} >
{{
static inline const char* run ()
{{
  return "{docstring}";
}}
static inline const char* attribute (const char* attrib)
{{{attributes}
  (void)attrib; // turn off unused parameter warning.
  return "";
}}
}};"""
template_class_attribute_body = """
  if (strcmp(attrib, "{attribute}") == 0)
    return "{docstring}";"""
template_constructor_doc = """
template <{tplargs}>
struct constructor_{nargs}_impl< {classname_prefix}{comma}{argsstring} >
{{
static inline const char* doc ()
{{
  return "{docstring}";
}}
static inline boost::python::detail::keywords<{nargs}+1> args ()
{{
  return ({argnamesstring});
}}
}};"""
template_destructor_doc = """
template <{tplargs}>
struct destructor_doc_impl < {classname_prefix} >
{{
static inline const char* run ()
{{
  return "{docstring}";
}}
}};"""
template_member_func_doc = """
{template}inline const char* member_func_doc ({rettype} ({classname_prefix}*function_ptr) {argsstring})
{{{body}
  return "";
}}"""
template_member_func_doc_body = """
  if (function_ptr == static_cast<{rettype} ({classname_prefix}*) {argsstring}>(&{classname_prefix}{membername}))
    return "{docstring}";"""
template_member_func_args = """
{template}inline boost::python::detail::keywords<{n}> member_func_args ({rettype} ({classname_prefix}*function_ptr) {argsstring})
{{{body}
  return ({default_args});
}}"""
template_member_func_args_body = """
  if (function_ptr == static_cast<{rettype} ({classname_prefix}*) {argsstring}>(&{classname_prefix}{membername}))
    return ({args});"""
template_static_func_doc = """
{template}inline const char* member_func_doc ({rettype} (*function_ptr) {argsstring})
{{{body}
  return "";
}}"""
template_static_func_doc_body = """
  if (function_ptr == static_cast<{rettype} (*) {argsstring}>(&{namespace}::{membername}))
    return "{docstring}";"""
template_open_namespace = """namespace {namespace} {{"""
template_close_namespace = """}} // namespace {namespace}"""
template_include_intern = """#include <doxygen_autodoc/{filename}>
"""
template_include_extern = """#include <{filename}>
"""


def _templateParamToDict(param):
    type_ = param.find("type")
    declname = param.find("declname")
    defname = param.find("defname")
    # FIXME type may contain references in two ways:
    # - the real param type
    # - the name of the template argument is recognized as the name of a type...
    if defname is None and declname is None:
        typetext = type_.text
        if typetext is None:
            typetext = ""
        for c in type_.iter():
            if c == type_:
                continue
            if c.text is not None:
                typetext += c.text
            if c.tail is not None:
                typetext += c.tail
        if typetext.startswith("typename") or typetext.startswith("class"):
            if sys.version_info.major == 2:
                s = typetext.split()
                return {"type": s[0].strip(), "name": typetext[len(s[0]) :].strip()}
            else:
                s = typetext.split(maxsplit=1)
                assert len(s) == 2
                return {"type": s[0].strip(), "name": s[1].strip()}
        else:
            return {"type": type_.text, "name": ""}
    else:
        if type_.text is None:
            # type_ is not a typename but an existing type
            type_ = type_.find("ref")
        assert defname.text == declname.text
        return {"type": type_.text, "name": defname.text}


def makeHeaderGuard(filename):
    return filename.upper().replace(".", "_").replace("/", "_").replace("-", "_")


def format_description(brief, detailed):
    b = [el.text.strip() for el in brief.iter() if el.text] if brief is not None else []
    d = (
        [el.text.strip() for el in detailed.iter() if el.text]
        if detailed is not None
        else []
    )
    text = "".join(b)
    if d:
        text += "\n" + "".join(d)
    return text


class Reference(object):
    def __init__(self, index, id=None, name=None):
        self.id = id
        self.name = name
        self.index = index

    def xmlToType(self, node, array=None, parentClass=None, tplargs=None):
        """
        - node:
        - parentClass: a class
        - tplargs: if one of the args is parentClass and no template arguments are provided,
                   set the template arguments to this value
        - array: content of the sibling tag 'array'
        """
        if node.text is not None:
            t = node.text.strip()
        else:
            t = ""
        for c in node.iterchildren():
            if c.tag == "ref":
                refid = c.attrib["refid"]
                if parentClass is not None and refid == parentClass.id:
                    t += " " + parentClass.name
                    if c.tail is not None and c.tail.lstrip()[0] != "<":
                        if tplargs is not None:
                            t += tplargs
                elif (
                    parentClass is not None
                    and isinstance(parentClass, ClassCompound)
                    and parentClass.hasTypeDef(c.text.strip())
                ):
                    parent_has_templates = len(parentClass.template_params) > 0
                    if parent_has_templates:
                        t += " typename " + parentClass._className() + "::"
                    else:
                        t += " " + parentClass._className() + "::"
                    self_has_templates = (
                        c.tail is not None and c.tail.strip().find("<") != -1
                    )
                    if self_has_templates:
                        t += " template "
                    t += c.text.strip()
                elif self.index.hasref(refid):
                    t += " " + self.index.getref(refid).name
                else:
                    self.index.output.warn("Unknown reference: ", c.text, refid)
                    t += " " + c.text.strip()
            else:
                if c.text is not None:
                    t += " " + c.text.strip()
            if c.tail is not None:
                t += " " + c.tail.strip()
        if array is not None:
            t += array.text
        return t


# Only for function as of now.
class MemberDef(Reference):
    def __init__(self, index, memberdefxml, parent):
        super(MemberDef, self).__init__(
            index=index,
            id=memberdefxml.attrib["id"],
            name=memberdefxml.find("definition").text,
        )
        self.parent = parent

        self.xml = memberdefxml
        self.const = memberdefxml.attrib["const"] == "yes"
        self.static = memberdefxml.attrib["static"] == "yes"
        self.rettype = memberdefxml.find("type")
        self.params = tuple(
            [
                (param.find("type"), param.find("declname"), param.find("array"))
                for param in self.xml.findall("param")
            ]
        )
        self.special = (
            self.rettype.text is None and len(self.rettype.getchildren()) == 0
        )
        # assert self.special or len(self.rettype.text) > 0

        self._templateParams(self.xml.find("templateparamlist"))

    def _templateParams(self, tpl):
        if tpl is not None:
            self.template_params = tuple(
                [_templateParamToDict(param) for param in tpl.iterchildren(tag="param")]
            )
        else:
            self.template_params = tuple()

    def prototypekey(self):
        prototype = (
            self.xmlToType(self.rettype, parentClass=self.parent),
            tuple([tuple(t.items()) for t in self.template_params]),
            tuple(
                [
                    self.xmlToType(param.find("type"), parentClass=self.parent)
                    for param in self.xml.findall("param")
                ]
            ),
            self.const,
        )
        return prototype

    def s_prototypeArgs(self):
        return "({0}){1}".format(self.s_args(), " const" if self.const else "")

    def s_args(self):
        # If the class is templated, check if one of the argument is the class itself.
        # If so, we must add the template arguments to the class (if there is none)

        if len(self.parent.template_params) > 0:
            tplargs = (
                " <"
                + ", ".join([d["name"] for d in self.parent.template_params])
                + " > "
            )
            args = ", ".join(
                [
                    self.xmlToType(
                        type, array, parentClass=self.parent, tplargs=tplargs
                    )
                    for type, declname, array in self.params
                ]
            )
        else:
            args = ", ".join(
                [
                    self.xmlToType(type, array, parentClass=self.parent)
                    for type, declname, array in self.params
                ]
            )
        return args

    def s_tpldecl(self):
        if len(self.template_params) == 0:
            return ""
        return ", ".join([d["type"] + " " + d["name"] for d in self.template_params])

    def s_rettype(self):
        assert not self.special, (
            "Member {} ({}) is a special function and no return type".format(
                self.name, self.id
            )
        )
        if len(self.parent.template_params) > 0:
            tplargs = (
                " <"
                + ", ".join([d["name"] for d in self.parent.template_params])
                + " > "
            )
        else:
            tplargs = None
        if isinstance(self.parent, ClassCompound):
            return self.xmlToType(
                self.rettype, parentClass=self.parent, tplargs=tplargs
            )
        else:
            return self.xmlToType(self.rettype)

    def s_name(self):
        return self.xml.find("name").text.strip()

    def s_docstring(self):
        return self.index.xml_docstring.getDocString(
            self.xml.find("briefdescription"),
            self.xml.find("detaileddescription"),
            self.index.output,
        )

    def n_args(self):
        return len(self.params)

    def s_argnamesstring(self):
        def getdeclname(i, declname):
            if declname is None or declname.text is None or declname.text.strip() == "":
                return "arg{}".format(i)
            return declname.text.strip()

        arg = """boost::python::arg("{}")"""
        argnames = [
            "self",
        ] + [getdeclname(i, declname) for i, (_, declname, _) in enumerate(self.params)]
        return ", ".join([arg.format(n) for n in argnames])

    def include(self):
        loc = self.xml.find("location")
        # The location is based on $CMAKE_SOURCE_DIR. Remove first directory.
        return loc.attrib["file"].split("/", 1)[1]


class CompoundBase(Reference):
    def __init__(self, compound, index):
        self.compound = compound
        self.filename = path.join(index.directory, compound.attrib["refid"] + ".xml")
        self.tree = etree.parse(self.filename)
        self.definition = self.tree.getroot().find("compounddef")
        super(CompoundBase, self).__init__(
            index,
            id=self.definition.attrib["id"],
            name=self.definition.find("compoundname").text,
        )


class NamespaceCompound(CompoundBase):
    def __init__(self, *args):
        super(NamespaceCompound, self).__init__(*args)
        self.typedefs = []
        self.enums = []
        self.static_funcs = []
        self.template_params = tuple()

        # Add references
        for section in self.definition.iterchildren("sectiondef"):
            assert "kind" in section.attrib
            kind = section.attrib["kind"]
            if kind == "enum":
                self.parseEnumSection(section)
            elif kind == "typedef":
                self.parseTypedefSection(section)
            elif kind == "func":
                self.parseFuncSection(section)

    def parseEnumSection(self, section):
        for member in section.iterchildren("memberdef"):
            ref = Reference(
                index=self.index,
                id=member.attrib["id"],
                name=self.name + "::" + member.find("name").text,
            )
            self.index.registerReference(ref)
            self.enums.append(member)
            for value in member.iterchildren("enumvalue"):
                ref = Reference(
                    index=self.index,
                    id=value.attrib["id"],
                    name=self.name + "::" + member.find("name").text,
                )

    def parseTypedefSection(self, section):
        for member in section.iterchildren("memberdef"):
            ref = Reference(
                index=self.index,
                id=member.attrib["id"],
                name=self.name + "::" + member.find("name").text,
            )
            self.index.registerReference(ref)
            self.typedefs.append(member)

    def parseFuncSection(self, section):
        for member in section.iterchildren("memberdef"):
            self.static_funcs.append(MemberDef(self.index, member, self))

    def innerNamespace(self):
        return self.name

    def write(self, output):
        pass


class ClassCompound(CompoundBase):
    def __init__(self, *args):
        super(ClassCompound, self).__init__(*args)
        self.member_funcs = list()
        self.static_funcs = list()
        self.special_funcs = list()
        self.attributes = list()

        self.struct = self.compound.attrib["kind"] == "struct"
        self.public = self.definition.attrib["prot"] == "public"
        self.template_specialization = self.name.find("<") > 0
        self.typedef = dict()

        # Handle templates
        self._templateParams(self.definition.find("templateparamlist"))
        for memberdef in self.definition.iter(tag="memberdef"):
            if memberdef.attrib["prot"] != "public":
                continue
            if memberdef.attrib["kind"] == "variable":
                self._attribute(memberdef)
            elif memberdef.attrib["kind"] == "typedef":
                ref = Reference(
                    index=self.index,
                    id=memberdef.attrib["id"],
                    name=self._className() + "::" + memberdef.find("name").text,
                )
                self.index.registerReference(ref)
                self.typedef[memberdef.find("name").text.strip()] = True

            elif memberdef.attrib["kind"] == "enum":
                if memberdef.find("name").text is None:
                    ref_name = self._className() + "::" + "anonymous_enum"
                else:
                    ref_name = self._className() + "::" + memberdef.find("name").text
                ref = Reference(
                    index=self.index,
                    id=memberdef.attrib["id"],
                    name=ref_name,
                )
                self.index.registerReference(ref)
                for value in memberdef.iterchildren("enumvalue"):
                    value_ref = Reference(
                        index=self.index,
                        id=value.attrib["id"],
                        name=ref.name,
                    )
                    self.index.registerReference(value_ref)
            elif memberdef.attrib["kind"] == "function":
                self._memberfunc(memberdef)

    def _templateParams(self, tpl):
        if tpl is not None:
            self.template_params = tuple(
                [_templateParamToDict(param) for param in tpl.iterchildren(tag="param")]
            )
        else:
            self.template_params = tuple()

    def _templateDecl(self):
        if not hasattr(self, "template_params") or len(self.template_params) == 0:
            return ""
        return ", ".join([d["type"] + " " + d["name"] for d in self.template_params])

    def _className(self):
        if not hasattr(self, "template_params") or len(self.template_params) == 0:
            return self.name
        return (
            self.name
            + " <"
            + ", ".join([d["name"] for d in self.template_params])
            + " >"
        )

    def hasTypeDef(self, typename):
        return typename in self.typedef

    def innerNamespace(self):
        return self._className()

    def _memberfunc(self, member):
        m = MemberDef(self.index, member, self)
        if m.special:
            self.special_funcs.append(m)
        elif m.static:
            self.static_funcs.append(m)
        else:
            self.member_funcs.append(m)

    def _writeClassDoc(self, output):
        docstring = self.index.xml_docstring.getDocString(
            self.definition.find("briefdescription"),
            self.definition.find("detaileddescription"),
            self.index.output,
        )
        attribute_docstrings = ""
        for member in self.attributes:
            _dc = self.index.xml_docstring.getDocString(
                member.find("briefdescription"),
                member.find("detaileddescription"),
                self.index.output,
            )
            if len(_dc) == 0:
                continue
            attribute_docstrings += template_class_attribute_body.format(
                attribute=member.find("name").text,
                docstring=_dc,
            )
        if len(docstring) == 0 and len(attribute_docstrings) == 0:
            return
        output.out(
            template_class_doc.format(
                tplargs=self._templateDecl(),
                classname=self._className(),
                docstring=docstring,
                attributes=attribute_docstrings,
            )
        )

    def write(self, output):
        if not self.public:
            return
        if self.template_specialization:
            output.warn(
                "Disable class {} because template argument are not resolved for templated class specialization.".format(
                    self.name
                )
            )
            return

        include = self.definition.find("includes")
        if include is None:
            output.err("Does not know where to write doc of", self.name)
            return
        output.open(include.text)
        output.out(template_include_extern.format(filename=include.text))
        output.out(template_open_namespace.format(namespace="doxygen"))

        # Write class doc
        self._writeClassDoc(output)

        # Group member function by prototype
        member_funcs = dict()
        for m in self.member_funcs:
            prototype = m.prototypekey()
            if prototype in member_funcs:
                member_funcs[prototype].append(m)
            else:
                member_funcs[prototype] = [
                    m,
                ]

        classname_prefix = self._className() + "::"

        for member in self.special_funcs:
            docstring = member.s_docstring()
            argnamesstring = member.s_argnamesstring()
            if len(docstring) == 0 and len(argnamesstring) == 0:
                continue
            if member.s_name()[0] == "~":
                output.out(
                    template_destructor_doc.format(
                        tplargs=self._templateDecl(),
                        classname_prefix=self._className(),
                        docstring=docstring,
                    )
                )
            else:
                output.out(
                    template_constructor_doc.format(
                        tplargs=", ".join(
                            [
                                d["type"] + " " + d["name"]
                                for d in self.template_params + member.template_params
                            ]
                        ),
                        nargs=len(member.params),
                        comma=", " if len(member.params) > 0 else "",
                        classname_prefix=self._className(),
                        argsstring=member.s_args(),
                        docstring=docstring,
                        argnamesstring=argnamesstring,
                    )
                )

        for prototype, members in member_funcs.items():
            # remove undocumented members
            documented_members = []
            docstrings = []
            argnamesstrings = []
            for member in members:
                docstring = member.s_docstring()
                argnamesstring = member.s_argnamesstring()
                if len(docstring) == 0 and len(argnamesstring) == 0:
                    continue
                documented_members.append(member)
                docstrings.append(docstring)
                argnamesstrings.append(argnamesstring)
            if len(documented_members) == 0:
                continue

            # Write docstrings
            body = "".join(
                [
                    template_member_func_doc_body.format(
                        classname_prefix=classname_prefix,
                        membername=member.s_name(),
                        docstring=docstring,
                        rettype=member.s_rettype(),
                        argsstring=member.s_prototypeArgs(),
                    )
                    for member, docstring in zip(documented_members, docstrings)
                ]
            )

            member = members[0]
            tplargs = ", ".join(
                [
                    d["type"] + " " + d["name"]
                    for d in self.template_params + member.template_params
                ]
            )
            output.out(
                template_member_func_doc.format(
                    template=(
                        "template <{}>\n".format(tplargs) if len(tplargs) > 0 else ""
                    ),
                    rettype=member.s_rettype(),
                    classname_prefix=classname_prefix,
                    argsstring=member.s_prototypeArgs(),
                    body=body,
                )
            )

            # Write argnamesstrings
            body = "".join(
                [
                    template_member_func_args_body.format(
                        classname_prefix=classname_prefix,
                        membername=member.s_name(),
                        args=argnamesstring,
                        rettype=member.s_rettype(),
                        argsstring=member.s_prototypeArgs(),
                    )
                    for member, argnamesstring in zip(
                        documented_members, argnamesstrings
                    )
                ]
            )

            n_args = member.n_args()

            default_args = ", ".join(
                [
                    """boost::python::arg("self")""",
                ]
                + ["""boost::python::arg("arg{}")""".format(i) for i in range(n_args)]
            )
            output.out(
                template_member_func_args.format(
                    template=(
                        "template <{}>\n".format(tplargs) if len(tplargs) > 0 else ""
                    ),
                    rettype=member.s_rettype(),
                    n=n_args + 1,
                    default_args=default_args,
                    classname_prefix=classname_prefix,
                    argsstring=member.s_prototypeArgs(),
                    body=body,
                )
            )

        output.out(template_close_namespace.format(namespace="doxygen"))
        output.close()

    def _attribute(self, member):
        self.attributes.append(member)


class Index:
    """
    This class is responsible for generating the list of all C++-usable documented elements.
    """

    def __init__(self, input, output):
        self.tree = etree.parse(input)
        self.directory = path.dirname(input)
        self.xml_docstring = XmlDocString(self)
        self.compounds = list()
        self.references = dict()
        self.output = output

    def parseCompound(self):
        for compound in self.tree.getroot().iterchildren("compound"):
            if compound.attrib["kind"] in ["class", "struct"]:
                obj = ClassCompound(compound, self)
            elif compound.attrib["kind"] == "namespace":
                obj = NamespaceCompound(compound, self)
            if obj.id not in self.compounds:
                self.compounds.append(obj.id)
            self.registerReference(obj)

    def write(self):
        # Header

        self.output.open("doxygen_xml_parser_for_cmake.hh")
        # self.output.out ("// Generated on {}".format (asctime()))
        self.output.close()

        # Implement template specialization for classes and member functions
        for id in self.compounds:
            compound = self.references[id]
            compound.write(self.output)

        self.output.open("functions.h")

        # Implement template specialization for static functions
        static_funcs = dict()
        prototypes = list()
        includes = list()
        for id in self.compounds:
            compound = self.references[id]
            for m in compound.static_funcs:
                include = m.include()
                if include not in includes:
                    includes.append(include)
                docstring = m.s_docstring()
                if len(docstring) == 0:
                    continue
                prototype = m.prototypekey()
                if prototype in static_funcs:
                    static_funcs[prototype].append((m, docstring))
                else:
                    static_funcs[prototype] = [
                        (m, docstring),
                    ]
                    prototypes.append(prototype)

        self.output.out(
            "".join(
                [
                    template_include_extern.format(filename=filename)
                    for filename in includes
                ]
            )
        )

        self.output.out(template_open_namespace.format(namespace="doxygen"))

        for prototype in prototypes:
            member_and_docstring_s = static_funcs[prototype]
            body = "".join(
                [
                    template_static_func_doc_body.format(
                        namespace=member.parent.innerNamespace(),
                        membername=member.s_name(),
                        docstring=docstring,
                        rettype=member.s_rettype(),
                        argsstring=member.s_prototypeArgs(),
                    )
                    for member, docstring in member_and_docstring_s
                ]
            )

            member = member_and_docstring_s[0][0]
            # TODO fix case of static method in templated class.
            tplargs = ", ".join(
                [
                    d["type"] + " " + d["name"]
                    for d in member.parent.template_params + member.template_params
                ]
            )
            self.output.out(
                template_static_func_doc.format(
                    template=(
                        "template <{}>\n".format(tplargs) if len(tplargs) > 0 else ""
                    ),
                    rettype=member.s_rettype(),
                    argsstring=member.s_prototypeArgs(),
                    body=body,
                )
            )

        self.output.out(template_close_namespace.format(namespace="doxygen"))
        self.output.close()

    def registerReference(self, obj, overwrite=True):
        if obj.id in self.references:
            if obj.name != self.references[obj.id].name:
                self.output.warn(
                    "!!!! Compounds " + obj.id + " already exists.",
                    obj.name,
                    self.references[obj.id].name,
                )
            else:
                self.output.warn("Reference " + obj.id + " already exists.", obj.name)
            if not overwrite:
                return
        self.references[obj.id] = obj

    def hasref(self, id):
        return id in self.references

    def getref(self, id):
        return self.references[id]


class OutputStreams(object):
    def __init__(self, output_dir, warn, error, errorPrefix=""):
        self.output_dir = output_dir
        self._out = None
        self._warn = warn
        self._err = error
        self.errorPrefix = errorPrefix

        self._created_files = dict()

    def open(self, name):
        assert self._out is None, "You did not close the previous file"
        import os

        fullname = os.path.join(self.output_dir, name)
        dirname = os.path.dirname(fullname)
        if not os.path.isdir(dirname):
            os.makedirs(dirname)

        if name in self._created_files:
            self._out = self._created_files[name]
        else:
            import codecs

            if sys.version_info >= (3,):
                encoding = "utf-8"
            else:
                encoding = "latin1"
            self._out = codecs.open(fullname, mode="w", encoding=encoding)
            self._created_files[name] = self._out

            # Header
            self.out(
                template_file_header.format(
                    path=os.path.dirname(os.path.abspath(__file__)),
                    header_guard=makeHeaderGuard(name),
                )
            )

    def close(self):
        self._out = None

    def writeFooterAndCloseFiles(self):
        for n, f in self._created_files.items():
            # Footer
            self._out = f
            self.out(
                template_file_footer.format(
                    header_guard=makeHeaderGuard(n),
                )
            )
            f.close()
        self._created_files.clear()
        self._out = None

    def out(self, *args):
        if sys.version_info >= (3,):
            print(*args, file=self._out)
        else:
            print(" ".join(str(arg) for arg in args).decode("latin1"), file=self._out)

    def warn(self, *args):
        print(self.errorPrefix, *args, file=self._warn)

    def err(self, *args):
        print(self.errorPrefix, *args, file=self._err)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Process Doxygen XML documentation and generate C++ code."
    )
    parser.add_argument("doxygen_index_xml", type=str, help="the Doxygen XML index.")
    parser.add_argument("output_directory", type=str, help="the output directory.")
    args = parser.parse_args()

    index = Index(
        input=sys.argv[1],
        output=OutputStreams(args.output_directory, sys.stdout, sys.stderr),
    )
    index.parseCompound()
    index.write()
    index.output.writeFooterAndCloseFiles()
    assert index.output._out is None
