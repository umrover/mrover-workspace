import os

# The root directory of the developer workspace
ROOT = os.path.dirname(os.path.dirname(__file__))

# The root directory of the build output
BUILD_ROOT = os.path.join(ROOT, 'build')

# The directory where intermediate build output is stored.
BUILD_INTERMEDIATE = os.path.join(ROOT, 'build', 'intermediate')

# The product venv directory
PRODUCT_ENV = os.path.join(BUILD_ROOT, 'env')

# The jarvis root directory
JARVIS_ROOT = os.path.join(ROOT, 'jarvis_files')

# The jarvis venv directory
JARVIS_ENV = os.path.join(JARVIS_ROOT, 'env')

# The store of source hashes
HASH_STORE = os.path.join(BUILD_ROOT, 'hashes')


# The directory containing third-party native libraries
THIRD_PARTY_ROOT = os.path.join(ROOT, '3rdparty')

# The directory containing LCM types
LCM_TYPES_ROOT = os.path.join(ROOT, 'rover_msgs')
