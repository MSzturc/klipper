# Code to implement asynchronous logging from a background thread
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, logging.handlers, threading, queue, time, os

# Class to forward all messages through a queue to a background thread
class QueueHandler(logging.Handler):
    def __init__(self, queue):
        logging.Handler.__init__(self)
        self.queue = queue
    def emit(self, record):
        try:
            self.format(record)
            record.msg = record.message
            record.args = None
            record.exc_info = None
            self.queue.put_nowait(record)
        except Exception:
            self.handleError(record)

# Class to poll a queue in a background thread and log each message
class QueueListener(logging.handlers.TimedRotatingFileHandler):
    def __init__(self, filename, rotate_log_at_restart=False):
        if rotate_log_at_restart:
            logging.handlers.TimedRotatingFileHandler.__init__(
                self, filename, when='S', interval=60 * 60 * 24,
                backupCount=5)
        else:
            logging.handlers.TimedRotatingFileHandler.__init__(
                self, filename, when='midnight', backupCount=5)
        self.bg_queue = queue.Queue()
        self.bg_thread = threading.Thread(target=self._bg_thread)
        self.bg_thread.start()
        self.rollover_info = {}
    def _bg_thread(self):
        while 1:
            record = self.bg_queue.get(True)
            if record is None:
                break
            self.handle(record)
    def stop(self):
        self.bg_queue.put_nowait(None)
        self.bg_thread.join()
    def set_rollover_info(self, name, info):
        if info is None:
            self.rollover_info.pop(name, None)
            return
        self.rollover_info[name] = info
    def clear_rollover_info(self):
        self.rollover_info.clear()
    def doRollover(self):
        # Automatic rotation entry point: called by the upstream
        # TimedRotatingFileHandler.emit() when shouldRollover() returns
        # True. The automatic case (midnight crossing for when='midnight'
        # or once-per-86400-seconds for when='S') only fires once per
        # suffix-resolution window, so upstream's archive naming is safe
        # here. LOG_ROLLOVER and the --rotate-log-at-restart restart
        # hook call manual_rollover() instead -- those paths can fire
        # repeatedly inside a single window.
        logging.handlers.TimedRotatingFileHandler.doRollover(self)
        self._emit_rollover_marker()

    def manual_rollover(self):
        # Collision-safe rollover for paths that may fire repeatedly
        # in a single suffix-resolution window (e.g. the LOG_ROLLOVER
        # gcode command run twice within the same day, or the
        # --rotate-log-at-restart startup-rollover and first
        # restart-loop rollover that land in the same second).
        # Upstream doRollover() derives the archive filename from
        # self.rolloverAt - self.interval, which yields the same
        # suffix string in those scenarios; upstream then
        # os.remove(dfn) the pre-existing archive, silently destroying
        # it. The implementation below derives the archive name from
        # the current time and appends a numeric counter on collision.
        self._do_unique_rollover()
        self._emit_rollover_marker()

    def _emit_rollover_marker(self):
        lines = [self.rollover_info[name]
                 for name in sorted(self.rollover_info)]
        lines.append(
            "=============== Log rollover at %s ===============" % (
                time.asctime(),))
        self.emit(logging.makeLogRecord(
            {'msg': "\n".join(lines), 'level': logging.INFO}))

    def _do_unique_rollover(self):
        if self.stream:
            self.stream.close()
            self.stream = None
        currentTime = int(time.time())
        timeTuple = (time.gmtime(currentTime) if self.utc
                     else time.localtime(currentTime))
        dfn = self.rotation_filename(
            self.baseFilename + "." + time.strftime(self.suffix, timeTuple))
        if os.path.exists(dfn):
            i = 1
            while os.path.exists("%s.%d" % (dfn, i)):
                i += 1
            dfn = "%s.%d" % (dfn, i)
        self.rotate(self.baseFilename, dfn)
        if self.backupCount > 0:
            for s in self.getFilesToDelete():
                os.remove(s)
        if not self.delay:
            self.stream = self._open()
        newRolloverAt = self.computeRollover(currentTime)
        while newRolloverAt <= currentTime:
            newRolloverAt = newRolloverAt + self.interval
        self.rolloverAt = newRolloverAt

MainQueueHandler = None

def setup_bg_logging(filename, debuglevel, rotate_log_at_restart=False):
    global MainQueueHandler
    ql = QueueListener(filename, rotate_log_at_restart=rotate_log_at_restart)
    MainQueueHandler = QueueHandler(ql.bg_queue)
    root = logging.getLogger()
    root.addHandler(MainQueueHandler)
    root.setLevel(debuglevel)
    return ql

def clear_bg_logging():
    global MainQueueHandler
    if MainQueueHandler is not None:
        root = logging.getLogger()
        root.removeHandler(MainQueueHandler)
        root.setLevel(logging.WARNING)
        MainQueueHandler = None
