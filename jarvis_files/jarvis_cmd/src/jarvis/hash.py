import os
import time
import itertools
from hashlib import sha256


class Hasher:
    def __init__(self, hash_store, name):
        self.hash_store = hash_store
        self.name = name
        self.hasher = sha256()


    def hash_modification_time(self, path):
        """
        Hashes the last modification time of all files and directories under path.
        """
        last_modification = 0
        if os.path.isdir(path):
            for root, dirs, files in os.walk(path, topdown=True, followlinks=False):
                paths = [os.path.join(root, name) for name in itertools.chain(dirs, files)]
                modifications = [os.stat(p).st_mtime for p in paths]
                modifications.append(last_modification)
                last_modification = max(modifications)
        else:
            last_modification = os.stat(path).st_mtime
        
        modification_string = time.ctime(last_modification)
        self.hasher.update(modification_string.encode('UTF-8'))


    def hash_build_options(self, opts):
        """
        Hashes the given build options, ignores order.
        """
        if opts is None:
            return

        opts.sort()
        for opt in opts:
            self.hasher.update(opt.encode('UTF-8'))


    def hash_lint(self, lint):
        """
        Hashes the given linting status.
        """
        self.hasher.update(str(lint).encode('UTF-8'))


    def has_changed(self):
        """
        Checks if the hash matches the saved version.
        """
        hash_file_path = os.path.join(self.hash_store, self.name)
        saved_hash = ''
        try:
            with open(hash_file_path) as hash_file:
                saved_hash = hash_file.read()
        except:
            pass

        return saved_hash != self.get_hash()


    def save(self):
        """
        Saves the hash to disk.
        """
        hash_file_path = os.path.join(self.hash_store, self.name)
        with open(hash_file_path, 'w') as hash_file:
            hash_file.write(self.get_hash())


    def get_hash(self):
        """
        Returns the hex value of the hash.
        """
        return self.hasher.hexdigest()

