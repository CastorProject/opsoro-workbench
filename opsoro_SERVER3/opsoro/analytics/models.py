from __future__ import unicode_literals

from django.db import models


class Resource(models.Model):
    # psutil.cpu_count()
    # cpu_count = models.PositiveIntegerField(default=0)
    # psutil.cpu_percent(interval=1, percpu=True)

    # psutil.cpu_freq(percpu=True)
    cpu = models.CharField(max_length=200, null=True, blank=True)

    # psutil.virtual_memory()
    # svmem(total=536870912, available=98533376, percent=81.6, used=424456192, free=0, active=100909056, inactive=336285696, buffers=0, cached=112414720, shared=154656768)
    virt_mem = models.CharField(max_length=250, null=True, blank=True)

    # psutil.swap_memory()
    # sswap(total=536870912, used=62943232, free=473927680, percent=11.7, sin=317214720, sout=549363712)
    swap_mem = models.CharField(max_length=200, null=True, blank=True)

    # psutil.disk_usage('/')
    # sdiskusage(total=21474836480, used=2597502976, free=18877333504, percent=12.1)
    disk = models.CharField(max_length=200, null=True, blank=True)

    timestamp = models.DateTimeField(auto_now_add=True, null=True, blank=True)

    def __str__(self):
        return str(self.timestamp)
