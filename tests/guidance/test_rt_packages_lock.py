"""Negative controls for tools/check_rt_packages_lock.py.

The verifier is the gate that stops an unpinned RT-Thread package selection from
being reported as reproducible. These cases feed synthetic (.config, lock)
pairs and require the specific problem to be reported; the positive control
shows a fully pinned pair is accepted.
"""
from pathlib import Path
import importlib.util
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'SConstruct').is_file())
MODULE_PATH = ROOT / 'tools/check_rt_packages_lock.py'
SPEC = importlib.util.spec_from_file_location('rt_packages_lock', MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

CONFIG_OK = (
    'CONFIG_PKG_USING_CMSIS_CORE=y\n'
    'CONFIG_PKG_CMSIS_CORE_VER="5.9.0"\n'
)
COMMIT = 'a' * 40
LOCK_OK = {
    'status': 'RESOLVED',
    'resolved_by': 'owner',
    'resolved_at_utc': '2026-11-10T00:00:00Z',
    'packages': [{
        'symbol': 'CMSIS_CORE',
        'pinned_version': '5.9.0',
        'pinned_commit': COMMIT,
    }],
}


def clone(lock):
    import copy
    return copy.deepcopy(lock)


def check(name, config, lock, expected):
    problems = MODULE.verify(config, lock)
    joined = ' | '.join(problems)
    if expected is None:
        if problems:
            print(f'FAIL {name}: expected no problems, got {joined}')
            return 1
        return 0
    if expected not in joined:
        print(f'FAIL {name}: expected {expected!r} in {joined!r}')
        return 1
    return 0


def main():
    failures = 0
    failures += check('positive', CONFIG_OK, clone(LOCK_OK), None)

    lock = clone(LOCK_OK)
    lock['status'] = 'UNRESOLVED'
    failures += check('unresolved', CONFIG_OK, lock, "expected 'RESOLVED'")

    lock = clone(LOCK_OK)
    lock['packages'][0]['pinned_version'] = 'latest'
    failures += check('latest version', CONFIG_OK, lock, 'no exact pinned_version')

    lock = clone(LOCK_OK)
    lock['packages'][0]['pinned_commit'] = 'abc123'
    failures += check('short commit', CONFIG_OK, lock, 'not a 40-hex commit')

    failures += check('latest flag still on',
                      CONFIG_OK + 'CONFIG_PKG_USING_CMSIS_CORE_LATEST_VERSION=y\n',
                      clone(LOCK_OK), 'LATEST_VERSION is still enabled')

    failures += check('version mismatch',
                      CONFIG_OK.replace('5.9.0', '5.8.0'),
                      clone(LOCK_OK), 'does not match pinned')

    lock = clone(LOCK_OK)
    lock['packages'].append({'symbol': 'EXTRA_PACKAGE',
                             'pinned_version': '1.0.0',
                             'pinned_commit': COMMIT})
    failures += check('extra lock entry', CONFIG_OK, lock, 'no longer selected')

    failures += check('missing lock entry',
                      CONFIG_OK + 'CONFIG_PKG_USING_OTHER=y\n'
                      'CONFIG_PKG_OTHER_VER="2.0.0"\n',
                      clone(LOCK_OK), 'absent from the lock')

    lock = clone(LOCK_OK)
    lock['resolved_by'] = ''
    failures += check('resolved without approver', CONFIG_OK, lock, 'resolved_by is empty')

    failures += check('missing version key',
                      'CONFIG_PKG_USING_CMSIS_CORE=y\n',
                      clone(LOCK_OK), 'VER is missing')

    if failures:
        print(f'FAIL: {failures} negative control(s) failed')
        return 1
    print('M5_RT_PACKAGES_LOCK_NEGATIVE_TEST_PASS (10 cases)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
