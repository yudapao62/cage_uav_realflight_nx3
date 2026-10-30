import hashlib
import os
import re
import shutil

from . import config


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def safe_name(name):
    name = os.path.basename(name or "unnamed")
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name)[:180] or "unnamed"


def immutable_copy(source, digest, extension):
    target = os.path.join(config.ORIGINALS, digest[:2], digest + extension.lower())
    os.makedirs(os.path.dirname(target), exist_ok=True)
    if not os.path.exists(target):
        temp = target + ".tmp"
        shutil.copyfile(source, temp)
        if sha256_file(temp) != digest:
            os.unlink(temp)
            raise ValueError("SOURCE_HASH_CHANGED")
        os.chmod(temp, 0o440)
        os.replace(temp, target)
    return target


def sniff_jpeg(path):
    with open(path, "rb") as handle:
        head = handle.read(3)
    # JPEG permits bytes after the EOI marker; structural decode is checked by Pillow.
    return head == b"\xff\xd8\xff"


def disk_has_room(path, incoming_size, reserve=1024 * 1024 * 1024):
    usage = shutil.disk_usage(path)
    return usage.free - incoming_size >= reserve
