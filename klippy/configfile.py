# Code for reading and writing the Klipper config file
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, glob, re, time, logging, configparser, io

error = configparser.Error


######################################################################
# Config section parsing helper
######################################################################

class sentinel:
    pass

class ConfigWrapper:
    error = configparser.Error
    def __init__(self, printer, fileconfig, access_tracking, section):
        self.printer = printer
        self.fileconfig = fileconfig
        self.access_tracking = access_tracking
        self.section = section
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.section
    def _get_wrapper(self, parser, option, default, minval=None, maxval=None,
                     above=None, below=None, note_valid=True):
        if not self.fileconfig.has_option(self.section, option):
            if default is not sentinel:
                if note_valid and default is not None:
                    acc_id = (self.section.lower(), option.lower())
                    self.access_tracking[acc_id] = default
                return default
            raise error("Option '%s' in section '%s' must be specified"
                        % (option, self.section))
        try:
            v = parser(self.section, option)
        except self.error as e:
            raise
        except:
            raise error("Unable to parse option '%s' in section '%s'"
                        % (option, self.section))
        if note_valid:
            self.access_tracking[(self.section.lower(), option.lower())] = v
        if minval is not None and v < minval:
            raise error("Option '%s' in section '%s' must have minimum of %s"
                        % (option, self.section, minval))
        if maxval is not None and v > maxval:
            raise error("Option '%s' in section '%s' must have maximum of %s"
                        % (option, self.section, maxval))
        if above is not None and v <= above:
            raise error("Option '%s' in section '%s' must be above %s"
                        % (option, self.section, above))
        if below is not None and v >= below:
            raise self.error("Option '%s' in section '%s' must be below %s"
                             % (option, self.section, below))
        return v
    def get(self, option, default=sentinel, note_valid=True):
        return self._get_wrapper(self.fileconfig.get, option, default,
                                 note_valid=note_valid)
    def getint(self, option, default=sentinel, minval=None, maxval=None,
               note_valid=True):
        return self._get_wrapper(self.fileconfig.getint, option, default,
                                 minval, maxval, note_valid=note_valid)
    def getfloat(self, option, default=sentinel, minval=None, maxval=None,
                 above=None, below=None, note_valid=True):
        return self._get_wrapper(self.fileconfig.getfloat, option, default,
                                 minval, maxval, above, below,
                                 note_valid=note_valid)
    def getboolean(self, option, default=sentinel, note_valid=True):
        return self._get_wrapper(self.fileconfig.getboolean, option, default,
                                 note_valid=note_valid)
    def getchoice(self, option, choices, default=sentinel, note_valid=True):
        if type(choices) == type([]):
            choices = {i: i for i in choices}
        if choices and type(list(choices.keys())[0]) == int:
            c = self.getint(option, default, note_valid=note_valid)
        else:
            c = self.get(option, default, note_valid=note_valid)
        if c not in choices:
            raise error("Choice '%s' for option '%s' in section '%s'"
                        " is not a valid choice" % (c, option, self.section))
        return choices[c]
    def getlists(self, option, default=sentinel, seps=(',',), count=None,
                 parser=str, note_valid=True):
        def lparser(value, pos):
            if len(value.strip()) == 0:
                # Return an empty list instead of [''] for empty string
                parts = []
            else:
                parts = [p.strip() for p in value.split(seps[pos])]
            if pos:
                # Nested list
                return tuple([lparser(p, pos - 1) for p in parts if p])
            res = [parser(p) for p in parts]
            if count is not None and len(res) != count:
                raise error("Option '%s' in section '%s' must have %d elements"
                            % (option, self.section, count))
            return tuple(res)
        def fcparser(section, option):
            return lparser(self.fileconfig.get(section, option), len(seps) - 1)
        return self._get_wrapper(fcparser, option, default,
                                 note_valid=note_valid)
    def getlist(self, option, default=sentinel, sep=',', count=None,
                note_valid=True):
        return self.getlists(option, default, seps=(sep,), count=count,
                             parser=str, note_valid=note_valid)
    def getintlist(self, option, default=sentinel, sep=',', count=None,
                   note_valid=True):
        return self.getlists(option, default, seps=(sep,), count=count,
                             parser=int, note_valid=note_valid)
    def getfloatlist(self, option, default=sentinel, sep=',', count=None,
                     note_valid=True):
        return self.getlists(option, default, seps=(sep,), count=count,
                             parser=float, note_valid=note_valid)
    def getsection(self, section):
        return ConfigWrapper(self.printer, self.fileconfig,
                             self.access_tracking, section)
    def has_section(self, section):
        return self.fileconfig.has_section(section)
    def get_prefix_sections(self, prefix):
        return [self.getsection(s) for s in self.fileconfig.sections()
                if s.startswith(prefix)]
    def get_prefix_options(self, prefix):
        return [o for o in self.fileconfig.options(self.section)
                if o.startswith(prefix)]
    def deprecate(self, option, value=None):
        if not self.fileconfig.has_option(self.section, option):
            return
        pconfig = self.printer.lookup_object("configfile")
        pconfig.deprecate(self.section, option, value)


######################################################################
# Variable interpolation and arithmetic evaluation
######################################################################

# Matches "${[section.]option[:default]}" placeholders that are NOT preceded
# by a backslash. The negative-lookbehind lets a config author emit a literal
# placeholder by writing "\${...}" (useful when a macro returns a string
# that contains "${...}").
_INTERPOLATION_KEYCRE = re.compile(
    r"(?<!\\)\$\{"
    r"(?:(?P<section>[^.:${}]+)[.:])?"
    r"(?P<option>[^${}:]+)"
    r"(?::(?P<default>[^{}]+))?"
    r"\}"
)

# Matches arithmetic expressions including the supported function calls.
_ARITHMETIC_PATTERN = re.compile(
    r'^[0-9.\+\-\*/\s]+$|^min\(.+\)$|^max\(.+\)$|^abs\(.+\)$|^round\(.+\)$'
)


class SectionInterpolation(configparser.Interpolation):
    """Variable interpolation of the form ${[section.]option[:default]}.

    After all placeholders are resolved, evaluates the result as an
    arithmetic expression if it matches one of the supported forms
    (numeric, min, max, abs, round).

    Authors can opt out of interpolation by escaping the leading dollar
    sign, e.g. "\\${something}", which is left as a literal "${something}"
    after interpolation.
    """

    def __init__(self, access_tracking):
        self.access_tracking = access_tracking
        # Tracks (section, option) tuples whose interpolation is currently in
        # flight. Resolving a placeholder calls back into parser.get(), which
        # re-enters before_get with a fresh depth counter; without this set a
        # cycle like a=${b}, b=${a} would recurse until Python raises
        # RecursionError instead of the documented InterpolationDepthError.
        self._in_progress = set()

    def before_get(self, parser, section, option, value, defaults):
        # configparser sometimes hands non-string defaults through here
        # (e.g. when an option has been set programmatically). Skip those
        # rather than crashing on the regex search.
        if not isinstance(value, str):
            return value

        key = (section, option)
        if key in self._in_progress:
            # Re-entry on the same option means the substitution chain has
            # looped. Raise the documented error instead of letting Python's
            # recursion limit blow up.
            raise configparser.InterpolationDepthError(option, section, value)

        is_outermost = not self._in_progress
        self._in_progress.add(key)
        try:
            depth = configparser.MAX_INTERPOLATION_DEPTH
            while depth:
                depth -= 1
                match = _INTERPOLATION_KEYCRE.search(value)
                if not match:
                    break

                sect = match.group("section") or section
                opt = match.group("option")
                dflt = match.group("default")

                try:
                    if (sect, opt) in self.access_tracking:
                        const = self.access_tracking[(sect, opt)]
                    else:
                        const = parser.get(sect, opt)
                except (configparser.NoSectionError,
                        configparser.NoOptionError):
                    if dflt is not None:
                        const = dflt
                    else:
                        raise

                value = value[: match.start()] + str(const) + value[match.end():]

            # If the loop exhausted all substitution steps and unresolved
            # placeholders remain, the chain is too deep.
            if _INTERPOLATION_KEYCRE.search(value):
                raise configparser.InterpolationDepthError(
                    option, section, value)
        finally:
            self._in_progress.discard(key)

        # Evaluate arithmetic on every frame so indirect references see the
        # numeric result rather than the raw expression — otherwise a sibling
        # like c=${b}*4 where b=1+2 would resolve to "1+2*4" (=9) instead of
        # "3*4" (=12) due to lost operator precedence.
        value = _evaluate_arithmetic_if_possible(value)

        # Cache the pre-strip, post-arithmetic form. Inner recursions that
        # consult access_tracking must see the literal "\${...}" so the
        # outer regex (which already excludes escaped placeholders) does not
        # re-interpret them — see the outermost-only escape-strip below.
        self.access_tracking.setdefault((section, option), value)

        # Strip the escape backslash only for the outermost interpolation.
        # If we did this on every recursion, an indirect reference like
        # a=${section.opt} where opt="\${foo}" would receive "${foo}" from
        # the inner call and the outer loop would re-interpret it as a
        # placeholder, breaking escape support through indirection.
        if is_outermost and "\\${" in value:
            value = value.replace("\\${", "${")
        return value


def _evaluate_arithmetic_if_possible(value):
    test_str = value.strip()
    if not _ARITHMETIC_PATTERN.match(test_str):
        return value
    try:
        if test_str.startswith("min(") and test_str.endswith(")"):
            args = _extract_arithmetic_args(test_str[4:-1])
            result = min(args)
        elif test_str.startswith("max(") and test_str.endswith(")"):
            args = _extract_arithmetic_args(test_str[4:-1])
            result = max(args)
        elif test_str.startswith("abs(") and test_str.endswith(")"):
            args = _extract_arithmetic_args(test_str[4:-1], single=True)
            result = abs(args[0])
        elif test_str.startswith("round(") and test_str.endswith(")"):
            args = _extract_arithmetic_args(test_str[6:-1], single=True)
            result = round(args[0])
        else:
            safe_globals = {"__builtins__": None}
            safe_locals = {}
            result = eval(test_str, safe_globals, safe_locals)
        if isinstance(result, (int, float)):
            return str(result)
        return value
    except Exception as e:
        logging.debug("Arithmetic evaluation failed for '%s': %s", value, e)
        return value


def _extract_arithmetic_args(arg_string, single=False):
    args = []
    for part in arg_string.split(','):
        part = part.strip()
        if _ARITHMETIC_PATTERN.match(part):
            args.append(float(_evaluate_arithmetic_if_possible(part)))
        else:
            raise ValueError("Invalid argument '%s' for operation." % (part,))
    if single and len(args) != 1:
        raise ValueError("Operation expected a single argument but got %d."
                         % (len(args),))
    return args


class ConfigNamespace:
    """Helper for conditional includes: exposes section options as
    attributes so an expression like "${stepper_x.enabled}" can be
    evaluated as Python code."""
    def __init__(self, data):
        for key, value in data.items():
            setattr(self, key, value)

    def __getitem__(self, item):
        return getattr(self, item)

    def __repr__(self):
        return str(self.__dict__)


######################################################################
# Config file parsing (with include file support)
######################################################################

# Used by ConfigFileReader._resolve_include() to detect a conditional
# include of the form "[include if:${expression} path/to/file.cfg]".
_CONDITIONAL_INCLUDE_RE = re.compile(r"if:\$\{(.+)\}\s+(.*)")


class ConfigFileReader:
    def read_config_file(self, filename):
        try:
            with open(filename, 'r') as f:
                data = f.read()
        except Exception as e:
            msg = "Unable to open config file %s: %s" % (filename, e)
            logging.exception(msg)
            raise error(msg)
        return data.replace('\r\n', '\n')
    def build_config_string(self, fileconfig):
        sfile = io.StringIO()
        fileconfig.write(sfile)
        return sfile.getvalue().strip()
    def append_fileconfig(self, fileconfig, data, filename):
        if not data:
            return
        # Strip trailing comments
        lines = data.split('\n')
        for i, line in enumerate(lines):
            pos = line.find('#')
            if pos >= 0:
                lines[i] = line[:pos]
        sbuffer = io.StringIO('\n'.join(lines))
        # Read into a temporary parser so we can apply printer.cfg-overrides
        # semantics: a value already present in fileconfig (set earlier in
        # the include chain or by the main printer.cfg) is NOT overwritten
        # by a later include. This makes the main printer.cfg the source of
        # truth and lets sub-configs declare defaults.
        temp_fileconfig = configparser.RawConfigParser(
            strict=False,
            inline_comment_prefixes=(';', '#'),
            interpolation=fileconfig._interpolation,
        )
        if sys.version_info.major >= 3:
            temp_fileconfig.read_file(sbuffer, filename)
        else:
            temp_fileconfig.readfp(sbuffer, filename)
        for section in temp_fileconfig.sections():
            if not fileconfig.has_section(section):
                fileconfig.add_section(section)
            for option in temp_fileconfig.options(section):
                if not fileconfig.has_option(section, option):
                    val = temp_fileconfig.get(section, option, raw=True)
                    fileconfig.set(section, option, val)
    def _create_fileconfig(self):
        access_tracking = {}
        if sys.version_info.major >= 3:
            fileconfig = configparser.RawConfigParser(
                strict=False,
                inline_comment_prefixes=(';', '#'),
                interpolation=SectionInterpolation(access_tracking),
            )
        else:
            fileconfig = configparser.RawConfigParser()
        return fileconfig
    def build_fileconfig(self, data, filename):
        fileconfig = self._create_fileconfig()
        self.append_fileconfig(fileconfig, data, filename)
        return fileconfig
    def build_fileconfig_with_includes(self, data, filename):
        fileconfig = self._create_fileconfig()
        self._parse_config(data, filename, fileconfig, set())
        # After all includes are resolved, expand placeholders so later
        # consumers see the final values. Entries that interpolate to the
        # literal string "None" are removed entirely (the "default value
        # 'None'" idiom for "no value provided").
        self._expand_all_values(fileconfig)
        return fileconfig
    def _expand_all_values(self, fileconfig):
        for section in fileconfig.sections():
            for option in fileconfig.options(section):
                val = fileconfig.get(section, option)
                if val == "None":
                    fileconfig.remove_option(section, option)
                elif "${" not in val:
                    # Persist the resolved value so consumers see the final
                    # form. Skip the write-back for values that still hold a
                    # literal "${...}" (originally escaped as "\${...}") — a
                    # second get() call would otherwise try to interpolate
                    # the literal and fail.
                    fileconfig.set(section, option, val)
    def _resolve_include(self, source_filename, include_spec, fileconfig,
                         visited):
        # Conditional include: "[include if:${expr} path]"
        condition_match = _CONDITIONAL_INCLUDE_RE.match(include_spec)
        if condition_match:
            expression, include_path = condition_match.groups()
            def convert_value(value):
                try:
                    if value.lower() == "true":
                        return True
                    if value.lower() == "false":
                        return False
                    if value.isdigit():
                        return int(value)
                    if "." in value:
                        return float(value)
                    return value
                except ValueError:
                    return value
            context = {
                section: ConfigNamespace(
                    {key: convert_value(value)
                     for key, value in fileconfig.items(section)})
                for section in fileconfig.sections()
            }
            try:
                condition_result = eval(expression,
                                        {"__builtins__": None}, context)
            except Exception as e:
                logging.warning("Failed to evaluate condition '%s': %s",
                                expression, e)
                condition_result = False
            if not condition_result:
                logging.info("Condition '%s' not met, skipping include %s",
                             expression, include_path)
                return None
        else:
            include_path = include_spec

        # Allow the include path itself to reference variables, so a config
        # can write "[include ${constants.profile}.cfg]".
        try:
            include_path = self._interpolate_include_path(include_path,
                                                          fileconfig)
        except ValueError as e:
            logging.warning("Failed to resolve interpolation in include "
                            "'%s': %s", include_spec, e)
            return None

        # Resolve relative include paths against the directory of the file
        # that issued the include, not the main printer.cfg directory.
        dirname = os.path.dirname(source_filename)
        include_glob = os.path.join(dirname, include_path)
        include_glob = os.path.abspath(include_glob)

        # Recursive glob lets sub-config trees be pulled in with a single
        # "[include sub/**/*.cfg]" line.
        include_filenames = glob.glob(include_glob, recursive=True)
        if not include_filenames and not glob.has_magic(include_glob):
            raise error("Include file '%s' does not exist" % (include_glob,))
        include_filenames.sort()
        for include_filename in include_filenames:
            include_data = self.read_config_file(include_filename)
            self._parse_config(include_data, include_filename, fileconfig,
                               visited)
        return None
    def _interpolate_include_path(self, value, fileconfig):
        # Bounded substitution loop: each pass replaces exactly one
        # placeholder, so the same ${section.option} appearing twice in
        # an include path (e.g. "[include ${A.x}/${A.x}.cfg]") resolves
        # in two passes without false-positive cycle detection. A genuine
        # cycle — an escaped self-reference like
        #   profile: \${constants.profile}
        #   [include ${constants.profile}.cfg]
        # — exhausts the depth counter (fileconfig.get() returns the
        # literal "${constants.profile}" since SectionInterpolation
        # strips the escape on the outermost call) and trips
        # InterpolationDepthError instead of looping forever.
        depth = configparser.MAX_INTERPOLATION_DEPTH
        while depth:
            depth -= 1
            match = _INTERPOLATION_KEYCRE.search(value)
            if not match:
                break
            sect = match.group("section") or "constants"
            opt = match.group("option")
            dflt = match.group("default")
            try:
                replacement = fileconfig.get(sect, opt)
            except (configparser.NoSectionError, configparser.NoOptionError):
                if dflt is not None:
                    replacement = dflt
                else:
                    raise ValueError(
                        "'%s.%s' not found and no default provided"
                        % (sect, opt))
            value = value[: match.start()] + replacement + value[match.end():]
        if _INTERPOLATION_KEYCRE.search(value):
            raise configparser.InterpolationDepthError(
                opt, sect, value)
        return value
    def _parse_config(self, data, filename, fileconfig, visited):
        path = os.path.abspath(filename)
        if path in visited:
            raise error("Recursive include of config file '%s'" % (filename,))
        visited.add(path)
        lines = data.split('\n')
        buf = []
        pending_includes = []
        for line in lines:
            # Strip trailing comment
            pos = line.find('#')
            if pos >= 0:
                line = line[:pos]
            mo = configparser.RawConfigParser.SECTCRE.match(line)
            header = mo and mo.group('header')
            if header and header.startswith('include '):
                # Defer include resolution until after the current file is
                # fully parsed so values declared in this file (typically
                # the main printer.cfg) win over identically-named values
                # in any included sub-config.
                pending_includes.append(header[8:].strip())
            else:
                buf.append(line)
        self.append_fileconfig(fileconfig, '\n'.join(buf), filename)
        for include_spec in pending_includes:
            self._resolve_include(filename, include_spec, fileconfig, visited)
        visited.remove(path)


######################################################################
# Config auto save helper
######################################################################

AUTOSAVE_HEADER = """
#*# <---------------------- SAVE_CONFIG ---------------------->
#*# DO NOT EDIT THIS BLOCK OR BELOW. The contents are auto-generated.
#*#
"""

class ConfigAutoSave:
    def __init__(self, printer):
        self.printer = printer
        self.fileconfig = None
        self.status_save_pending = {}
        self.save_config_pending = False
        gcode = self.printer.lookup_object('gcode')
        # Guard against double-registration so RELOAD_GCODE_MACROS can
        # rebuild PrinterConfig without crashing on the SAVE_CONFIG handler
        # already being present.
        if "SAVE_CONFIG" not in gcode.ready_gcode_handlers:
            gcode.register_command("SAVE_CONFIG", self.cmd_SAVE_CONFIG,
                                   desc=self.cmd_SAVE_CONFIG_help)
    def _find_autosave_data(self, data):
        regular_data = data
        autosave_data = ""
        pos = data.find(AUTOSAVE_HEADER)
        if pos >= 0:
            regular_data = data[:pos]
            autosave_data = data[pos + len(AUTOSAVE_HEADER):].strip()
        # Check for errors and strip line prefixes
        if "\n#*# " in regular_data or autosave_data.find(AUTOSAVE_HEADER) >= 0:
            logging.warning("Can't read autosave from config file"
                            " - autosave state corrupted")
            return data, ""
        out = [""]
        for line in autosave_data.split('\n'):
            if ((not line.startswith("#*#")
                 or (len(line) >= 4 and not line.startswith("#*# ")))
                and autosave_data):
                logging.warning("Can't read autosave from config file"
                                " - modifications after header")
                return data, ""
            out.append(line[4:])
        out.append("")
        return regular_data, "\n".join(out)
    comment_r = re.compile('[#;].*$')
    value_r = re.compile('[^A-Za-z0-9_].*$')
    def _strip_duplicates(self, data, fileconfig):
        # Comment out fields in 'data' that are defined in 'config'
        lines = data.split('\n')
        section = None
        is_dup_field = False
        for lineno, line in enumerate(lines):
            pruned_line = self.comment_r.sub('', line).rstrip()
            if not pruned_line:
                continue
            if pruned_line[0].isspace():
                if is_dup_field:
                    lines[lineno] = '#' + lines[lineno]
                continue
            is_dup_field = False
            if pruned_line[0] == '[':
                section = pruned_line[1:-1].strip()
                continue
            field = self.value_r.sub('', pruned_line)
            if fileconfig.has_option(section, field):
                is_dup_field = True
                lines[lineno] = '#' + lines[lineno]
        return "\n".join(lines)
    def load_main_config(self):
        filename = self.printer.get_start_args()['config_file']
        cfgrdr = ConfigFileReader()
        data = cfgrdr.read_config_file(filename)
        regular_data, autosave_data = self._find_autosave_data(data)
        regular_fileconfig = cfgrdr.build_fileconfig_with_includes(
            regular_data, filename)
        autosave_data = self._strip_duplicates(autosave_data,
                                               regular_fileconfig)
        self.fileconfig = cfgrdr.build_fileconfig(autosave_data, filename)
        cfgrdr.append_fileconfig(regular_fileconfig,
                                 autosave_data, '*AUTOSAVE*')
        return regular_fileconfig, self.fileconfig
    def get_status(self, eventtime):
        return {'save_config_pending': self.save_config_pending,
                'save_config_pending_items': self.status_save_pending}
    def set(self, section, option, value):
        if not self.fileconfig.has_section(section):
            self.fileconfig.add_section(section)
        svalue = str(value)
        self.fileconfig.set(section, option, svalue)
        pending = dict(self.status_save_pending)
        if not section in pending or pending[section] is None:
            pending[section] = {}
        else:
            pending[section] = dict(pending[section])
        pending[section][option] = svalue
        self.status_save_pending = pending
        self.save_config_pending = True
        logging.info("save_config: set [%s] %s = %s", section, option, svalue)
    def remove_section(self, section):
        if self.fileconfig.has_section(section):
            self.fileconfig.remove_section(section)
            pending = dict(self.status_save_pending)
            pending[section] = None
            self.status_save_pending = pending
            self.save_config_pending = True
        elif (section in self.status_save_pending and
              self.status_save_pending[section] is not None):
            pending = dict(self.status_save_pending)
            del pending[section]
            self.status_save_pending = pending
            self.save_config_pending = True
    def _disallow_include_conflicts(self, regular_fileconfig):
        for section in self.fileconfig.sections():
            for option in self.fileconfig.options(section):
                if regular_fileconfig.has_option(section, option):
                    msg = ("SAVE_CONFIG section '%s' option '%s' conflicts "
                           "with included value" % (section, option))
                    raise self.printer.command_error(msg)
    cmd_SAVE_CONFIG_help = "Overwrite config file and restart"
    def cmd_SAVE_CONFIG(self, gcmd):
        if not self.fileconfig.sections():
            return
        # Create string containing autosave data
        cfgrdr = ConfigFileReader()
        autosave_data = cfgrdr.build_config_string(self.fileconfig)
        lines = [('#*# ' + l).strip()
                 for l in autosave_data.split('\n')]
        lines.insert(0, "\n" + AUTOSAVE_HEADER.rstrip())
        lines.append("")
        autosave_data = '\n'.join(lines)
        # Read in and validate current config file
        cfgname = self.printer.get_start_args()['config_file']
        try:
            data = cfgrdr.read_config_file(cfgname)
        except error as e:
            msg = "Unable to read existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        regular_data, old_autosave_data = self._find_autosave_data(data)
        regular_data = self._strip_duplicates(regular_data, self.fileconfig)
        data = regular_data.rstrip() + autosave_data
        new_regular_data, new_autosave_data = self._find_autosave_data(data)
        if not new_autosave_data:
            raise gcmd.error(
                "Existing config autosave is corrupted."
                " Can't complete SAVE_CONFIG")
        try:
            regular_fileconfig = cfgrdr.build_fileconfig_with_includes(
                new_regular_data, cfgname)
        except error as e:
            msg = "Unable to parse existing config on SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        self._disallow_include_conflicts(regular_fileconfig)
        # Determine filenames
        datestr = time.strftime("-%Y%m%d_%H%M%S")
        backup_name = cfgname + datestr
        temp_name = cfgname + "_autosave"
        if cfgname.endswith(".cfg"):
            backup_name = cfgname[:-4] + datestr + ".cfg"
            temp_name = cfgname[:-4] + "_autosave.cfg"
        # Create new config file with temporary name and swap with main config
        logging.info("SAVE_CONFIG to '%s' (backup in '%s')",
                     cfgname, backup_name)
        try:
            f = open(temp_name, 'w')
            f.write(data)
            f.close()
            os.rename(cfgname, backup_name)
            os.rename(temp_name, cfgname)
        except:
            msg = "Unable to write config file during SAVE_CONFIG"
            logging.exception(msg)
            raise gcmd.error(msg)
        # Request a restart
        gcode = self.printer.lookup_object('gcode')
        gcode.request_restart('restart')


######################################################################
# Config validation (check for undefined options)
######################################################################

# Section name reserved for declaring constants used by ${constants.X}
# placeholders. Has no runtime object so the unused-options check skips it.
CONSTANTS_SECTION = 'constants'


class ConfigValidate:
    def __init__(self, printer):
        self.printer = printer
        self.status_settings = {}
        self.access_tracking = {}
        self.autosave_options = {}
    def start_access_tracking(self, autosave_fileconfig):
        # Note autosave options for use during undefined options check
        self.autosave_options = {}
        for section in autosave_fileconfig.sections():
            for option in autosave_fileconfig.options(section):
                self.autosave_options[(section.lower(), option.lower())] = 1
        self.access_tracking = {}
        return self.access_tracking
    def check_unused(self, fileconfig):
        # Don't warn on fields set in autosave segment
        access_tracking = dict(self.access_tracking)
        access_tracking.update(self.autosave_options)
        # Note locally used sections
        valid_sections = { s: 1 for s, o in self.printer.lookup_objects() }
        valid_sections.update({ s: 1 for s, o in access_tracking })
        # Validate that there are no undefined parameters in the config file
        for section_name in fileconfig.sections():
            section = section_name.lower()
            if section == CONSTANTS_SECTION:
                continue
            if section not in valid_sections:
                raise error("Section '%s' is not a valid config section"
                            % (section,))
            for option in fileconfig.options(section_name):
                option = option.lower()
                if (section, option) not in access_tracking:
                    raise error("Option '%s' is not valid in section '%s'"
                                % (option, section))
        # Setup get_status()
        self._build_status_settings()
        # Clear tracking state
        self.access_tracking.clear()
        self.autosave_options.clear()
    def _build_status_settings(self):
        self.status_settings = {}
        for (section, option), value in self.access_tracking.items():
            self.status_settings.setdefault(section, {})[option] = value
    def get_status(self, eventtime):
        return {'settings': self.status_settings}


######################################################################
# Main printer config tracking
######################################################################

class PrinterConfig:
    def __init__(self, printer):
        self.printer = printer
        self.autosave = ConfigAutoSave(printer)
        self.validate = ConfigValidate(printer)
        self.deprecated = {}
        self.status_raw_config = {}
        self.status_warnings = []
    def get_printer(self):
        return self.printer
    def read_config(self, filename):
        cfgrdr = ConfigFileReader()
        data = cfgrdr.read_config_file(filename)
        fileconfig = cfgrdr.build_fileconfig(data, filename)
        return ConfigWrapper(self.printer, fileconfig, {}, 'printer')
    def read_main_config(self):
        fileconfig, autosave_fileconfig = self.autosave.load_main_config()
        access_tracking = self.validate.start_access_tracking(
            autosave_fileconfig)
        config = ConfigWrapper(self.printer, fileconfig,
                               access_tracking, 'printer')
        self._build_status_config(config)
        return config
    def log_config(self, config):
        cfgrdr = ConfigFileReader()
        lines = ["===== Config file =====",
                 cfgrdr.build_config_string(config.fileconfig),
                 "======================="]
        self.printer.set_rollover_info("config", "\n".join(lines))
    def check_unused_options(self, config):
        self.validate.check_unused(config.fileconfig)
    # Deprecation warnings
    def _add_deprecated(self, data):
        key = tuple(list(data.items()))
        if key in self.deprecated:
            return False
        self.deprecated[key] = True
        self.status_warnings = self.status_warnings + [data]
        return True
    def runtime_warning(self, msg):
        res = {'type': 'runtime_warning', 'message': msg}
        did_add = self._add_deprecated(res)
        if did_add:
            logging.warning(msg)
    def warn(self, type, msg, section=None, option=None, value=None):
        res = {'type': type, 'message': msg}
        if section is not None:
            res['section'] = section
        if option is not None:
            res['option'] = option
        if value is not None:
            res['value'] = value
        did_add = self._add_deprecated(res)
        if did_add:
            logging.warning(msg)
    def deprecate(self, section, option, value=None, msg=None):
        if value is None:
            defmsg = ("Option '%s' in section '%s' is deprecated."
                   % (option, section))
            self.warn('deprecated_option', msg or defmsg, section, option)
        else:
            defmsg = ("Value '%s' in option '%s' in section '%s' is deprecated."
                      % (value, option, section))
            self.warn('deprecated_value', msg or defmsg,
                      section, option, value)
    def deprecate_gcode(self, cmd, param=None, value=None, msg=None):
        if param is None:
            defmsg = "Command '%s' is deprecated." % (cmd,)
        elif value is None:
            defmsg = ("Parameter '%s' in command '%s' is deprecated."
                      % (param, cmd))
        else:
            defmsg = ("Value '%s=%s' in command '%s' is deprecated."
                      % (param, value, cmd))
        if msg is None:
            msg = defmsg
        res = {'type': 'deprecated_gcode', 'message': msg,
               'command': cmd, 'parameter': param, 'value': str(value)}
        self._add_deprecated(res)
    def deprecate_mcu_code(self, mcu, feature, msg=None):
        mcu_name = mcu.get_name()
        if msg is None:
            vhost = self.printer.start_args['software_version']
            vmcu = mcu.get_status()['mcu_version']
            msg = ("MCU '%s' has deprecated code (it is missing feature '%s')."
                   " Recompiling and flashing is recommended (MCU version '%s',"
                   " host version '%s')." % (mcu_name, feature, vmcu, vhost))
        res = {'type': 'deprecated_mcu_code', 'message': msg,
               'mcu': mcu_name, 'feature': feature}
        self._add_deprecated(res)
    # Status reporting
    def _build_status_config(self, config):
        self.status_raw_config = {}
        for section in config.get_prefix_sections(''):
            self.status_raw_config[section.get_name()] = section_status = {}
            for option in section.get_prefix_options(''):
                section_status[option] = section.get(option, note_valid=False)
    def get_status(self, eventtime):
        status = {'config': self.status_raw_config,
                  'warnings': self.status_warnings}
        status.update(self.autosave.get_status(eventtime))
        status.update(self.validate.get_status(eventtime))
        return status
    # Autosave functions
    def set(self, section, option, value):
        self.autosave.set(section, option, value)
    def remove_section(self, section):
        self.autosave.remove_section(section)
