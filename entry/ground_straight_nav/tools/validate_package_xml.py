#!/usr/bin/env python3
"""Validate every package.xml under a catkin workspace the way catkin_pkg does.

A single invalid manifest makes ``catkin_find`` raise, and the exception takes
roslaunch down for the *whole* workspace -- including packages that are
perfectly fine.  That is what happened with ground_straight_nav 20261018_v1:
version "20261018_v1" is rejected because underscores are not allowed.

No ROS installation is needed; this mirrors the checks in catkin_pkg.package
that matter in practice (version convention, maintainer email, required tags).
Run it before installing anything into a workspace you also use for other work.

    python3 tools/validate_package_xml.py /home/abot/abot_ws/src
    python3 tools/validate_package_xml.py --self
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

# catkin_pkg.package.check_licenses / _validate_version
VERSION_RE = re.compile(
    r'^[0-9]+\.[0-9]+\.[0-9]+'
    r'(?:[-_.]?(?:a|b|c|rc|alpha|beta|pre|preview)[-_.]?[0-9]*)?'
    r'(?:-[0-9]+|[-_.]?(?:post|rev|r)[-_.]?[0-9]*)?'
    r'(?:[-_.]?dev[-_.]?[0-9]*)?$')
# catkin_pkg.package._EMAIL_RE
EMAIL_RE = re.compile(
    r'^[a-zA-Z0-9.!#$%&\'*+/=?^_`{|}~-]+'
    r'@[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?'
    r'(?:\.[a-zA-Z0-9](?:[a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?)+$')
NAME_RE = re.compile(r'^[a-z][a-z0-9_-]*$')


def check_package_xml(path):
    """Return a list of (severity, message). Empty list means valid."""
    problems = []
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        return [('error', 'XML 解析失败: %s' % exc)]
    if root.tag != 'package':
        problems.append(('error', '根元素必须是 <package>，实际是 <%s>' % root.tag))

    def text(tag):
        node = root.find(tag)
        return (node.text or '').strip() if node is not None else ''

    name = text('name')
    if not name:
        problems.append(('error', '缺少 <name>'))
    elif not NAME_RE.match(name):
        problems.append(('error', '包名 %r 只允许小写字母、数字、下划线、连字符，'
                                  '且以小写字母开头' % name))

    version = text('version')
    if not version:
        problems.append(('error', '缺少 <version>'))
    elif not VERSION_RE.match(version):
        problems.append(('error', '版本 "%s" 不符合版本规范；需要形如 1.0.0，'
                                  '下划线不被接受' % version))

    for tag in ('maintainer', 'license', 'description'):
        if not text(tag):
            problems.append(('error', '缺少 <%s>' % tag))

    maintainers = root.findall('maintainer')
    if not maintainers:
        problems.append(('error', '缺少 <maintainer>'))
    # Only maintainer emails are validated: catkin_pkg accepts <author>
    # without an email, and plenty of shipped ROS packages rely on that.
    for person in maintainers:
        email = (person.get('email') or '').strip()
        who = (person.text or '').strip() or '?'
        if not email:
            problems.append(('error', 'maintainer %r 缺少 email 属性' % who))
        elif not EMAIL_RE.match(email):
            problems.append(('error', 'maintainer 非法邮箱 "%s"（%r）；'
                                      'localhost 这类无点域名不被接受'
                            % (email, who)))

    format_attr = root.get('format') or '1'
    if format_attr not in ('1', '2', '3'):
        problems.append(('error', 'format="%s" 不是 1/2/3' % format_attr))

    buildtools = [n.text.strip() for n in root.findall('buildtool_depend') if n.text]
    if not buildtools:
        problems.append(('error', '缺少 <buildtool_depend>（通常为 catkin）'))
    unknown = [d for d in ('depend', 'build_depend', 'build_export_depend',
                           'exec_depend', 'run_depend', 'test_depend',
                           'buildtool_depend', 'doc_depend', 'buildtool_export_depend')
               if root.findall(d)]
    if not unknown:
        problems.append(('warn', '没有任何依赖声明'))
    return problems


def scan(workspace_src, skip=()):
    total = 0
    broken = 0
    for dirpath, dirnames, filenames in os.walk(workspace_src):
        dirnames[:] = [d for d in dirnames
                       if not d.startswith('.') and d not in ('devel', 'build')]
        if 'package.xml' not in filenames:
            continue
        rel = os.path.relpath(os.path.join(dirpath, 'package.xml'), workspace_src)
        if any(rel.startswith(s) for s in skip):
            continue
        total += 1
        path = os.path.join(dirpath, 'package.xml')
        problems = check_package_xml(path)
        errors = [m for s, m in problems if s == 'error']
        if errors:
            broken += 1
            print('INVALID  %s' % rel)
            for message in errors:
                print('         %s' % message)
        else:
            print('ok       %s' % rel)
    print()
    print('共 %d 个包，%d 个非法' % (total, broken))
    return 0 if broken == 0 else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace_src', nargs='?',
                        help='工作空间的 src 目录，例如 /home/abot/abot_ws/src')
    parser.add_argument('--self', action='store_true',
                        help='只检查本包自己的 package.xml')
    parser.add_argument('--skip', default='',
                        help='跳过的相对路径前缀，逗号分隔')
    args = parser.parse_args(argv)

    if args.self:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path = os.path.join(here, 'package.xml')
        problems = check_package_xml(path)
        if not problems:
            print('ok  %s' % path)
            return 0
        for severity, message in problems:
            print('%-5s %s' % (severity.upper(), message))
        return 1

    if not args.workspace_src:
        parser.error('需要 workspace_src，或者用 --self')
    skip = tuple(s for s in args.skip.split(',') if s)
    return scan(os.path.abspath(args.workspace_src), skip)


if __name__ == '__main__':
    sys.exit(main())
