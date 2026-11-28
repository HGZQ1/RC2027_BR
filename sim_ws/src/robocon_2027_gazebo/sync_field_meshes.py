#!/usr/bin/env python3
"""检查 Gazebo 场地素材，缺少时从原发布版本补齐。"""

import argparse
import shutil
import tarfile
import tempfile
import urllib.request
from pathlib import Path
from xml.etree import ElementTree

VERSION = '4be62dd'
ARCHIVE_ROOT = f'ABU-Robocon-2027-Gazebo-{VERSION}'
ARCHIVE_URL = f'https://codeload.github.com/YaoZA233/ABU-Robocon-2027-Gazebo/tar.gz/{VERSION}'
MODEL = 'robocon_2027_field'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, help='已下载的原版 tar.gz，离线补齐时使用')
    args = parser.parse_args()
    model_dir = Path(__file__).resolve().parent / MODEL
    prefix = f'model://{MODEL}/'
    sdf = ElementTree.parse(model_dir / 'model.sdf')
    relative_paths = {
        Path(uri.text.strip().removeprefix(prefix))
        for uri in sdf.findall('.//mesh/uri')
        if uri.text and uri.text.strip().startswith(prefix)
    }
    missing = sorted(path for path in relative_paths if not (model_dir / path).is_file())
    if not missing:
        print(f'场地网格齐全（{len(relative_paths)} 个）。')
        return

    with tempfile.TemporaryDirectory(prefix='rc2027-field-') as folder:
        archive = args.archive
        if archive is None:
            archive = Path(folder) / 'upstream.tar.gz'
            with urllib.request.urlopen(ARCHIVE_URL, timeout=60) as response, archive.open('wb') as out:
                shutil.copyfileobj(response, out)
        with tarfile.open(archive, 'r:gz') as source:
            members = [source.getmember(f'{ARCHIVE_ROOT}/{MODEL}/{path.as_posix()}')
                       for path in missing]
            if not all(member.isfile() for member in members):
                raise ValueError('上游压缩包内的网格必须是普通文件')
            for path, member in zip(missing, members):
                target = model_dir / path
                target.parent.mkdir(parents=True, exist_ok=True)
                with source.extractfile(member) as mesh, target.open('xb') as out:
                    shutil.copyfileobj(mesh, out)
    print(f'已补齐 {len(missing)} 个网格。场地共需要 {len(relative_paths)} 个。')


if __name__ == '__main__':
    main()
