import sys
import time
from multiprocessing import Process, Queue, Event, freeze_support
from TeamControl.process_workers.vision_runner import VisionProcess
from TeamControl.process_workers.gcfsm_runner import GCfsm
from TeamControl.world.model_manager import WorldModelManager
from TeamControl.process_workers.wm_runner import WMWorker
from TeamControl.SSL.grSim.sandbox_process import run_grsim_sandbox_process
from TeamControl.bt.run_bt_v2_process import run_bt_v2_process

# in multiprocessing this can only be a simple process

def main():
    freeze_support()
    vision_port = 10006
    is_running = Event()
    is_running.set()
    vision_q = Queue()
    gc_q = Queue()
    dispatcher_q = Queue()

    config_file = sys.argv[1] if len(sys.argv) > 1 else "ipconfig.yaml"
    preset = Config(config_file)

    wm_manager = WorldModelManager()
    wm_manager.start()
    wm = wm_manager.WorldModel()
    wmr = Process(target=WMWorker.run_worker, args=(is_running,None,wm,vision_q,gc_q,))
    sandbox = Process(target=run_grsim_sandbox_process, args=(wm,) )
    # new version
    bt = Process(target=run_bt_v2_process, args=(is_running, wm, dispatcher_q,))

    vision_wkr.start()
    wmr.start()
    # sandbox.start()
    bt.start()

    # Watchdog: print a warning if any process dies unexpectedly
    try:
        while is_running.is_set():
            for p in (vision_wkr, gc_wkr, wmr, bt, dispatcher):
                if not p.is_alive():
                    print(f"[sandbox] WARNING: process '{p.name}' died", flush=True)
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("[sandbox] Shutting down...", flush=True)
        is_running.clear()

    for p in (vision_wkr, gc_wkr, wmr, bt, dispatcher):
        p.join(timeout=5)


if __name__ == "__main__":
    main()