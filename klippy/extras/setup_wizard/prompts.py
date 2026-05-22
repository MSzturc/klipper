# action:prompt rendering helpers. Klipper emits each returned line via
# gcode.respond_info(), which prepends "// " so Mainsail/Fluidd render the
# modal dialog. Recommended option = primary style + position 1.


def dialog(title, text=None, buttons=None, footer=None):
    """Return the ordered list of action:prompt_* directives."""
    lines = ["action:prompt_begin %s" % title]
    if text:
        lines.append("action:prompt_text %s" % text)
    for label, gcode, style in (buttons or []):
        lines.append("action:prompt_button %s|%s|%s" % (label, gcode, style))
    for label, gcode, style in (footer or []):
        lines.append("action:prompt_footer_button %s|%s|%s"
                     % (label, gcode, style))
    lines.append("action:prompt_show")
    return lines
