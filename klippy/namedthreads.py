import ctypes
import ctypes.util
import threading
import logging
import os

# this is mostly copied from https://github.com/beniwohli/namedthreads/blob/master/namedthreads.py
# itself copied from https://bugs.python.org/issue15500#msg230736
def make_pthread_setname_np():
    libpthread_path = ctypes.util.find_library("pthread")
    if not libpthread_path:
        # pthread library not found, not patching"
        return
    libpthread = ctypes.CDLL(libpthread_path)
    if not hasattr(libpthread, "pthread_setname_np"):
        # pthread library does not have pthread_setname_np function, not patching
        return
    pthread_setname_np = libpthread.pthread_setname_np
    pthread_setname_np.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    pthread_setname_np.restype = ctypes.c_int
    return pthread_setname_np
pthread_setname_np = make_pthread_setname_np()
def set_thread_name(name, thread=None):
    if hasattr(name, "encode"):
        name = name.encode('ascii', 'replace')
    if thread is None:
        thread = threading.current_thread()
    ident = getattr(thread, "ident", None)
    ret = -1
    if ident is not None:
        ret = pthread_setname_np(ident, name[:15])
    if ret != 0:
        raise RuntimeError(f'Failed to set thread name {thread=} {ident=} {name=}')
def patch():
    if getattr(threading.Thread.start, "_namedthreads_patched", None):
        # threading module is already patched
        return
    if pthread_setname_np is None:
        logging.warn("Failed to get pthread_setname_np() wrapper")
        return
    orig_start = threading.Thread.start
    def start(self):
        name = self.name

        if name == 'klippy_serial':
            os.setpriority(os.PRIO_PROCESS,0,-16)

        if name == 'klippy_logger':
            os.setpriority(os.PRIO_PROCESS,0,-12)

        orig_start(self)
        try:
            if name:
                set_thread_name(name, self)
        except Exception as e:
            logging.warn("Failed to set thread name for %r: %s", self, e)
        os.setpriority(os.PRIO_PROCESS,0,-14)

    start._namedthreads_patched = True
    start._namedthreads_orig = threading.Thread.start
    threading.Thread.start = start
    return True
def unpatch():
    if not getattr(threading.Thread.start, "_namedthreads_patched", None):
        # threading module is not patched
        return
    patched_start = threading.Threading.start
    threading.Thread.start = patched_start._namedthreads_orig