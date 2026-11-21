"""Slow log handlers must not stall control or lock out input callbacks."""
import ast
import threading
import unittest
from test_auto_transport import app


class NonBlockingLogTests(unittest.TestCase):
    def test_slow_logger_does_not_block_producer_or_control_lock(self):
        entered=threading.Event();release=threading.Event();done=threading.Event()
        lock=threading.RLock()
        def slow(*args):
            entered.set();release.wait(3)
        log=app.NonBlockingLog(slow,capacity=4)
        try:
            with log:
                log.info('first')
                self.assertTrue(entered.wait(1))
                def control():
                    for i in range(1000):
                        with lock:log.info('tick %s',i)
                    done.set()
                producer=threading.Thread(target=control,daemon=True)
                producer.start()
                self.assertTrue(done.wait(1),'control producer stalled on slow logger')
                self.assertTrue(lock.acquire(timeout=.1))
                lock.release()
                self.assertEqual(log.queue.qsize(),4)
                self.assertEqual(log.dropped,996)
            self.assertTrue(log.closed.is_set())
            self.assertTrue(log.worker.is_alive(),'shutdown must not wait on logger')
        finally:
            release.set();log.worker.join(1)

    def test_handler_failure_does_not_kill_worker(self):
        second=threading.Event()
        def logger(value):
            if value=='bad':raise OSError('broken console')
            second.set()
        with app.NonBlockingLog(logger) as log:
            log.info('bad');log.info('good')
            self.assertTrue(second.wait(1))
            self.assertEqual(log.errors,1)
        log.worker.join(1)

    def test_run_loop_uses_async_text_logging_but_keeps_control_outputs(self):
        tree=ast.parse((app.ROOT/'scripts/auto_window.py').read_text())
        runtime=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Runtime')
        run=next(n for n in runtime.body if isinstance(n,ast.FunctionDef) and n.name=='run')
        def name(n):
            if isinstance(n,ast.Name):return n.id
            if isinstance(n,ast.Attribute):return name(n.value)+'.'+n.attr
            return ''
        loop=next(n for n in ast.walk(run) if isinstance(n,ast.While) and any(
            isinstance(x,ast.Attribute) and x.attr=='is_shutdown' for x in ast.walk(n.test)))
        calls=[name(n.func) for n in ast.walk(loop) if isinstance(n,ast.Call)]
        self.assertNotIn('self.ros.loginfo',calls)
        self.assertIn('run_log.info',calls)
        for name in ('self.mission.tick','self.publish','self.diag.publish','self.request'):
            self.assertIn(name,calls)
