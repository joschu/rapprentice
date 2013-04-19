import logging as _logging

log = _logging.getLogger("rapprentice")
log.setLevel(_logging.DEBUG)

_ch = _logging.StreamHandler()
_ch.setLevel(_logging.DEBUG)
_formatter = _logging.Formatter('%(levelname)s - %(message)s')
_ch.setFormatter(_formatter)
log.addHandler(_ch)

