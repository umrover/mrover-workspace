from invoke import task
from circus import get_arbiter

import config

from . import dep


ONBOARD_PROGRAMS = [
        'onboard_teleop',
        'nav',
        # TODO add hwi
]


@task(dep)
def onboard(ctx, rover='hughey'):
    """
    Launches a supervisor for the onboard software stack. (Defaults to Hughey)
    """
    arbiter = get_arbiter({
        'cmd': p,
        'numprocesses': 1,
        'virtualenv': config.PRODUCT_ENV,
        'copy_env': True
    } for p in ONBOARD_PROGRAMS)
    try:
        arbiter.start()
    finally:
        arbiter.stop()
