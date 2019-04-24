import psutil
from django_cron import CronJobBase, Schedule

from analytics.models import Resource

try:
    import simplejson as json
except ImportError:
    import json


class check_usage(CronJobBase):
    RUN_EVERY_MINS = 1  # every 2 hours

    schedule = Schedule(run_every_mins=RUN_EVERY_MINS)
    code = 'opsoro.check_usage'    # a unique code

    def do(self):
        # [2, 20]
        cpu = str(json.dumps(psutil.cpu_percent(percpu=True)))
        # svmem(total=536870912, available=98533376, percent=81.6, used=424456192, free=0, active=100909056, inactive=336285696, buffers=0, cached=112414720, shared=154656768)
        virt_mem = str(json.dumps(psutil.virtual_memory()))
        # sswap(total=536870912, used=62943232, free=473927680, percent=11.7, sin=317214720, sout=549363712)
        swap_mem = str(json.dumps(psutil.swap_memory()))
        # sdiskusage(total=21474836480, used=2597502976, free=18877333504, percent=12.1)
        disk = str(json.dumps(psutil.disk_usage('/')))

        # print(cpu, virt_mem, swap_mem, disk)

        usag = Resource.objects.create()
        usag.cpu = cpu
        usag.virt_mem = virt_mem
        usag.swap_mem = swap_mem
        usag.disk = disk
        usag.save()
