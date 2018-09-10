"""Functions used to generate source files during build time
All such functions are invoked in a subprocess on Windows to prevent build flakiness.
"""
from platform_methods import subprocess_main
import re

class LegacyGLHeaderStruct:

    def __init__(self):
        self.vertex_lines = []
        self.fragment_lines = []
        self.attributes = []
        self.feedbacks = []
        self.fbos = []
        self.conditionals = []
        self.enums = {}
        self.texunits = []
        self.texunit_names = []
        self.ubos = []
        self.ubo_names = []

        self.vertex_included_files = []
        self.fragment_included_files = []

        self.reading = ""
        self.line_offset = 0
        self.vertex_offset = 0
        self.fragment_offset = 0


def include_file_in_header(filename, header_data, depth):
    fs = open(filename, "r")
    line = fs.readline()
    while line:

        if line.find("[vertex]") != -1:
            header_data.reading = "vertex"
            line = fs.readline()
            header_data.line_offset += 1
            header_data.vertex_offset = header_data.line_offset
            continue

        if line.find("[fragment]") != -1:
            header_data.reading = "fragment"
            line = fs.readline()
            header_data.line_offset += 1
            header_data.fragment_offset = header_data.line_offset
            continue

        while line.find("#include ") != -1:
            includeline = line.replace("#include ", "").strip()[1:-1]

            import os.path

            included_file = os.path.relpath(os.path.dirname(filename) + "/" + includeline)
            if not included_file in header_data.vertex_included_files and header_data.reading == "vertex":
                header_data.vertex_included_files += [included_file]
                if include_file_in_header(included_file, header_data, depth + 1) == None:
                    print("Error in file '" + filename + "': #include " + includeline + "could not be found!")
            elif not included_file in header_data.fragment_included_files and header_data.reading == "fragment":
                header_data.fragment_included_files += [included_file]
                if include_file_in_header(included_file, header_data, depth + 1) == None:
                    print("Error in file '" + filename + "': #include " + includeline + "could not be found!")

            line = fs.readline()

        if line.find("#ifdef ") != -1 or line.find("#elif defined(") != -1:
            if line.find("#ifdef ") != -1:
                ifdefline = line.replace("#ifdef ", "").strip()
            else:
                ifdefline = line.replace("#elif defined(", "").strip()
                ifdefline = ifdefline.replace(")", "").strip()

            if line.find("_EN_") != -1:
                enumbase = ifdefline[:ifdefline.find("_EN_")]
                ifdefline = ifdefline.replace("_EN_", "_")
                line = line.replace("_EN_", "_")
                if enumbase not in header_data.enums:
                    header_data.enums[enumbase] = []
                if ifdefline not in header_data.enums[enumbase]:
                    header_data.enums[enumbase].append(ifdefline)

            elif not ifdefline in header_data.conditionals:
                header_data.conditionals += [ifdefline]

        if line.find("uniform") != -1 and line.lower().find("texunit:") != -1:
            # texture unit
            texunitstr = line[line.find(":") + 1:].strip()
            if texunitstr == "auto":
                texunit = "-1"
            else:
                texunit = str(int(texunitstr))
            uline = line[:line.lower().find("//")]
            uline = uline.replace("uniform", "")
            uline = uline.replace("highp", "")
            uline = uline.replace(";", "")
            lines = uline.split(",")
            for x in lines:

                x = x.strip()
                x = x[x.rfind(" ") + 1:]
                if x.find("[") != -1:
                    # unfiorm array
                    x = x[:x.find("[")]

                if not x in header_data.texunit_names:
                    header_data.texunits += [(x, texunit)]
                    header_data.texunit_names += [x]

        elif line.find("uniform") != -1 and line.lower().find("ubo:") != -1:
            pass

        if line.strip().find("attribute ") == 0 and line.find("attrib:") != -1:
            uline = line.replace("in ", "")
            uline = uline.replace("attribute ", "")
            uline = uline.replace("highp ", "")
            uline = uline.replace(";", "")
            uline = uline[uline.find(" "):].strip()

            if uline.find("//") != -1:
                name, bind = uline.split("//")
                if bind.find("attrib:") != -1:
                    name = name.strip()
                    bind = bind.replace("attrib:", "").strip()
                    header_data.attributes += [(name, bind)]

        if line.strip().find("out ") == 0 and line.find("tfb:") != -1:
            uline = line.replace("out ", "")
            uline = uline.replace("highp ", "")
            uline = uline.replace(";", "")
            uline = uline[uline.find(" "):].strip()

            if uline.find("//") != -1:
                name, bind = uline.split("//")
                if bind.find("tfb:") != -1:
                    name = name.strip()
                    bind = bind.replace("tfb:", "").strip()
                    header_data.feedbacks += [(name, bind)]

        line = line.replace("\r", "")
        line = line.replace("\n", "")

        if header_data.reading == "vertex":
            header_data.vertex_lines += [line]
        if header_data.reading == "fragment":
            header_data.fragment_lines += [line]

        line = fs.readline()
        header_data.line_offset += 1

    fs.close()

    fs = open(filename, "r")
    line = fs.readline()

    #Read the entire file to find the ends of UBO
    while line:
        if line.find("uniform") != -1 and line.lower().find("texunit:") != -1:
            pass
        elif line.find("uniform") != -1 and line.find("{") == -1 and line.find(";") != -1:
            pass
        elif line.find("uniform") != -1 and line.lower().find("ubo:") != -1:
            # uniform buffer object
            ubostr = line[line.find(":") + 1:].strip()
            ubo = str(int(ubostr))
            uline = line[:line.lower().find("//")]
            uline = uline[uline.find("uniform") + len("uniform"):]
            uline = uline.replace("highp", "")
            uline = uline.replace(";", "")
            uline = uline.replace("{", "").strip()
            lines = uline.split(",")
            for x in lines:
                x = x.strip()
                x = x[x.rfind(" ") + 1:]
                if x.find("[") != -1:
                    # uniform array
                    x = x[:x.find("[")]
                ubo_lines = []
                ubo_line = fs.readline()
                if not x in header_data.ubo_names:
                    while ubo_line.find("}") == -1:                    
                       ubo_line = ubo_line.split("//")[0]
                       if ubo_line.startswith("//"):
                           ubo_line = ""                    
                       ubo_line = ubo_line.replace("highp", "")
                       ubo_line = ubo_line.replace("mediump", "")
                       ubo_line = ubo_line.replace(";", "")
                       ubo_line = ubo_line.strip()
                       if ubo_line != "":
                           ubo_lines += [ubo_line]
                       ubo_line = fs.readline()
                    header_data.ubos += [(x, ubo, ubo_lines)]
                    header_data.ubo_names += [x]

        line = fs.readline()
        header_data.line_offset += 1

    fs.close()


    return header_data


def build_vulkan_header(filename, include, class_suffix, output_attribs):
    header_data = LegacyGLHeaderStruct()
    include_file_in_header(filename, header_data, 0)

    out_file = filename + ".gen.h"
    fd = open(out_file, "w")

    enum_constants = []

    fd.write("/* WARNING, THIS FILE WAS GENERATED, DO NOT EDIT */\n")

    out_file_base = out_file
    out_file_base = out_file_base[out_file_base.rfind("/") + 1:]
    out_file_base = out_file_base[out_file_base.rfind("\\") + 1:]
    out_file_ifdef = out_file_base.replace(".", "_").upper()
    fd.write("#ifndef " + out_file_ifdef + "_" + class_suffix.upper() + "_450\n")
    fd.write("#define " + out_file_ifdef + "_" + class_suffix.upper() + "_450\n")

    out_file_class = out_file_base.replace(".glsl.gen.h", "").title().replace("_", "").replace(".", "") + "Shader" + class_suffix
    fd.write("\n\n")
    fd.write("#include \"" + include + "\"\n\n\n")
    fd.write("class " + out_file_class + " : public Shader" + class_suffix + " {\n\n")
    fd.write("\t virtual String get_shader_name() const { return \"" + out_file_class + "\"; }\n")

    fd.write("public:\n\n")

    if header_data.conditionals:
        fd.write("\tenum Conditionals {\n")
        for x in header_data.conditionals:
            fd.write("\t\t" + x.upper() + ",\n")
        fd.write("\t};\n\n")

    if header_data.conditionals:
        fd.write("\t_FORCE_INLINE_ void set_conditional(Conditionals p_conditional,bool p_enable)  {  _set_conditional(p_conditional,p_enable); }\n\n")

    fd.write("\tvirtual void init() {\n\n")

    enum_value_count = 0

    if header_data.enums:

        fd.write("\t\t//Written using math, given nonstandarity of 64 bits integer constants..\n")
        fd.write("\t\tstatic const Enum _enums[]={\n")

        bitofs = len(header_data.conditionals)
        enum_vals = []

        for xv in header_data.enums:
            x = header_data.enums[xv]
            bits = 1
            amt = len(x)
            while (2 ** bits < amt):
                bits += 1
            strs = "{"
            for i in range(amt):
                strs += "\"#define " + x[i] + "\\n\","

                c = {}
                c["set_mask"] = "uint64_t(" + str(i) + ")<<" + str(bitofs)
                c["clear_mask"] = "((uint64_t(1)<<40)-1) ^ (((uint64_t(1)<<" + str(bits) + ") - 1)<<" + str(bitofs) + ")"
                enum_vals.append(c)
                enum_constants.append(x[i])

            strs += "NULL}"

            fd.write("\t\t\t{(uint64_t(1<<" + str(bits) + ")-1)<<" + str(bitofs) + "," + str(bitofs) + "," + strs + "},\n")
            bitofs += bits

        fd.write("\t\t};\n\n")

        fd.write("\t\tstatic const EnumValue _enum_values[]={\n")

        enum_value_count = len(enum_vals)
        for x in enum_vals:
            fd.write("\t\t\t{" + x["set_mask"] + "," + x["clear_mask"] + "},\n")

        fd.write("\t\t};\n\n")

    conditionals_found = []
    if header_data.conditionals:

        fd.write("\t\tstatic const char* _conditional_strings[]={\n")
        if header_data.conditionals:
            for x in header_data.conditionals:
                fd.write("\t\t\t\"#define " + x + "\\n\",\n")
                conditionals_found.append(x)
        fd.write("\t\t};\n\n")
    else:
        fd.write("\t\tstatic const char **_conditional_strings=NULL;\n")

    if output_attribs:
        if header_data.attributes:
            fd.write("\t\tstatic AttributePair _attribute_pairs[]={\n")
            for x in header_data.attributes:
                fd.write("\t\t\t{\"" + x[0] + "\"," + x[1] + "},\n")
            fd.write("\t\t};\n\n")
        else:
            fd.write("\t\tstatic AttributePair *_attribute_pairs=NULL;\n")

    feedback_count = 0

    if len(header_data.feedbacks):

        fd.write("\t\tstatic const Feedback _feedbacks[]={\n")
        for x in header_data.feedbacks:
            name = x[0]
            cond = x[1]
            if cond in conditionals_found:
                fd.write("\t\t\t{\"" + name + "\"," + str(conditionals_found.index(cond)) + "},\n")
            else:
                fd.write("\t\t\t{\"" + name + "\",-1},\n")

            feedback_count += 1

        fd.write("\t\t};\n\n")
    else:
        fd.write("\t\tstatic const Feedback* _feedbacks=NULL;\n")

    if header_data.texunits:
        fd.write("\t\tstatic TexUnitPair _texunit_pairs[]={\n")
        for x in header_data.texunits:
            fd.write("\t\t\t{\"" + x[0] + "\"," + x[1] + "},\n")
        fd.write("\t\t};\n\n")
    else:
        fd.write("\t\tstatic TexUnitPair *_texunit_pairs=NULL;\n")

    if header_data.ubos:
        fd.write("\t\tstatic UBOPair _ubo_pairs[]={\n")
        for x in header_data.ubos:
            fd.write("\t\t\t{\"" + x[0] + "\"," + x[1] + "},\n")
        fd.write("\t\t};\n\n")
    else:
        fd.write("\t\tstatic UBOPair *_ubo_pairs=NULL;\n")

    fd.write("\t\tstatic const char _vertex_code[]={\n")
    for x in header_data.vertex_lines:
        for c in x:
            fd.write(str(ord(c)) + ",")

        fd.write(str(ord('\n')) + ",")
    fd.write("\t\t0};\n\n")

    fd.write("\t\tstatic const int _vertex_code_start=" + str(header_data.vertex_offset) + ";\n")

    fd.write("\t\tstatic const char _fragment_code[]={\n")
    for x in header_data.fragment_lines:
        for c in x:
            fd.write(str(ord(c)) + ",")

        fd.write(str(ord('\n')) + ",")
    fd.write("\t\t0};\n\n")

    fd.write("\t\tstatic const int _fragment_code_start=" + str(header_data.fragment_offset) + ";\n")

    if output_attribs:
        fd.write("\t\tsetup(_conditional_strings," + str(len(header_data.conditionals)) + ",_attribute_pairs," + str(
            len(header_data.attributes)) + ", _texunit_pairs," + str(len(header_data.texunits)) + ",_ubo_pairs," + str(len(header_data.ubos)) + ",_feedbacks," + str(
            feedback_count) + ",_vertex_code,_fragment_code,_vertex_code_start,_fragment_code_start);\n")
    else:
        fd.write("\t\tsetup(_conditional_strings," + str(len(header_data.conditionals)) + ",_uniform_strings," + str(len(header_data.uniforms)) + ",_texunit_pairs," + str(
            len(header_data.texunits)) + ",_enums," + str(len(header_data.enums)) + ",_enum_values," + str(enum_value_count) + ",_ubo_pairs," + str(len(header_data.ubos)) + ",_feedbacks," + str(
            feedback_count) + ",_vertex_code,_fragment_code,_vertex_code_start,_fragment_code_start);\n")

    fd.write("\t}\n\n")

    if enum_constants:

        fd.write("\tenum EnumConditionals {\n")
        for x in enum_constants:
            fd.write("\t\t" + x.upper() + ",\n")
        fd.write("\t};\n\n")
        fd.write("\tvoid set_enum_conditional(EnumConditionals p_cond) { _set_enum_conditional(p_cond); }\n")

    fd.write("public:\n")
    if header_data.ubos:
        for x in header_data.ubos:
            offset = 0;
            fd.write("\tstruct " + x[0] + " {\n")
            i = 0
            for uniform in x[2]:
                uniform_list = re.split(r'\s+', uniform.strip())
                if len(uniform_list) > 1:                
                    uniform_type = uniform_list[0]
                    uniform_name = uniform_list[1]
                    if get_datatype_c(uniform_type):
                        datatype_c = get_datatype_c(uniform_type)[0]
                        array = ""
                        if get_datatype_c(uniform_type):
                            array_count = get_datatype_c(uniform_type)[1]
                            if array_count > 1:
                                array = "[" + str(array_count) + "]"
                        fd.write("\t\t" + datatype_c + " " + uniform_name + array + ";\n")
                        offset += get_datatype_alignment(uniform_type)
                        align = get_datatype_size(uniform_type) % get_datatype_alignment(uniform_type)
                        if align != 0:
                            padding = get_datatype_alignment(uniform_type) - align
                            pad = int(padding / 4)
                            fd.write( "\t\tfloat align_" + str(i) + "[" + str(pad) + "];\n" )
                            i += 1
                    
            if offset % 16 != 0: #UBO sizes must be multiples of 16
                align = offset % 16
                padding = 16 - align
                pad = int(padding / 4)
                fd.write("\t\tfloat _pad[" + str(pad) + "];\n" )

            fd.write("\t};\n\n")

    fd.write("};\n\n")
    fd.write("#endif\n\n")
    fd.close()

def build_vulkan_headers(target, source, env):
    for x in source:
        build_vulkan_header(str(x), include="drivers/vulkan/shader_vulkan.h", class_suffix="Vulkan", output_attribs=True)

def get_datatype_c(p_type):
	switcher = {
		"void": ["void", 1],
		"bool": ["bool", 1],
		"bvec2": ["bool", 2],
		"bvec3": ["bool", 3],
		"bvec4": ["bool", 4],
		"int": ["int32_t", 1],
		"ivec2": ["int32_t", 2],
		"ivec3": ["int32_t", 3],
		"ivec4": ["int32_t", 4],
		"uint": ["uint32_t", 1],
		"uvec2": ["uint32_t", 2],
		"uvec3": ["uint32_t", 3],
		"uvec4": ["uiint32_t", 4],
		"float": ["float", 1],
		"vec2": ["float", 2],
		"vec3": ["float", 3],
		"vec4": ["float", 4],
		"mat2": ["float", 4],
		"mat3":	["float", 9],
		"mat4": ["float", 16]
	}
	return switcher.get(p_type)
    
def get_datatype_size(p_type):
	switcher = {
		"void": 0,
		"bool": 4,
		"bvec2": 8,
		"bvec3": 12,
		"bvec4": 16,
		"int": 4,
		"ivec2": 8,
		"ivec3": 12,
		"ivec4": 16,
		"uint": 4,
		"uvec2": 8,
		"uvec3": 12,
		"uvec4": 16,
		"float": 4,
		"vec2": 8,
		"vec3": 12,
		"vec4": 16,
		"mat2":
			32, #4 * 4 + 4 * 4
		"mat3":
			48, #4 * 4 + 4 * 4 + 4 * 4
		"mat4": 64,
		"sampler2D": 16,
		"isampler2D": 16,
		"usampler2D": 16,
		"samplerCube": 16
	}
	return switcher.get(p_type)

def get_datatype_alignment(p_type):
	switcher = {
		"void": 0,
		"bool": 4,
		"bvec2": 8,
		"bvec3": 16,
		"bvec4": 16,
		"int": 4,
		"ivec2": 8,
		"ivec3": 16,
		"ivec4": 16,
		"uint": 4,
		"uvec2": 8,
		"uvec3": 16,
		"uvec4": 16,
		"float": 4,
		"vec2": 8,
		"vec3": 16,
		"vec4": 16,
		"mat2": 16,
		"mat3": 16,
		"mat4": 16,
		"sampler2D": 16,
		"isampler2D": 16,
		"usampler2D": 16,
		"samplerCube": 16
	}

	return switcher.get(p_type)

if __name__ == '__main__':
    subprocess_main(globals())

