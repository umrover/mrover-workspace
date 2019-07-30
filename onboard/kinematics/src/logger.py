import logging


logging.basicConfig(filename='/tmp/mrover-kinematics.log', level=logging.DEBUG)

logging.getLogger("imported_module").setLevel(logging.WARNING)
# create logger
logger = logging.getLogger('mrover_arm')
logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

# create formatter
f = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
formatter = logging.Formatter(f)

# add formatter to ch
ch.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)
