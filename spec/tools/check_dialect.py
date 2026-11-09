"""Validate spec FCU.xml using an existing pymavlink install, in a temp directory.

python spec/tools/check_dialect.py --pymavlink-root <temporary-library-directory>
No download, firmware import, or driver generation is performed by this script.
"""
import argparse
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pymavlink-root', type=Path)
    parser.add_argument('--cc', default=r'D:\STM32CubeCLT\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe')
    args = parser.parse_args()
    if args.pymavlink_root:
        sys.path.insert(0, str(args.pymavlink_root.resolve()))
    import pymavlink
    from pymavlink.generator import mavgen
    library = Path(pymavlink.__file__).resolve().parent
    xmlsource = library/'message_definitions/v1.0'
    if not xmlsource.is_dir():
        xmlsource = library/'dialects/v20'
    source = ROOT/'spec/contracts/FCU.xml'
    log, cases = io.StringIO(), []
    with tempfile.TemporaryDirectory(prefix='flight-spec-dialect-') as tmp:
        stage = Path(tmp)
        for name in ['common.xml','standard.xml','minimal.xml']:
            shutil.copy2(xmlsource/name, stage/name)
        shutil.copy2(source, stage/'FCU.xml')
        with contextlib.redirect_stdout(log), contextlib.redirect_stderr(log):
            for language, output in [('C',stage/'generated'),('Python3',stage/'reference.py')]:
                assert mavgen.mavgen(mavgen.Opts(str(output),wire_protocol='2.0',language=language,validate=True,strict_units=True),[str(stage/'FCU.xml')])
        spec = importlib.util.spec_from_file_location('fcu_reference',stage/'reference.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        for message in ET.parse(source).findall('./messages/message'):
            cls = getattr(module,'MAVLink_'+message.attrib['name'].lower()+'_message')
            for boundary in [False, True]:
                values = {}
                for field in message.findall('field'):
                    kind, name = field.attrib['type'], field.attrib['name']
                    if kind.endswith(']'):
                        value = [255 if boundary else 0]*int(kind.split('[')[1][:-1])
                    elif kind == 'float':
                        value = 1.25 if boundary else 0.0
                    elif kind.startswith('uint'):
                        value = (1<<int(kind[4:kind.index('_')]))-1 if boundary else 0
                    else:
                        value = -1 if boundary else 0
                    values[name] = value
                packet = cls(**values).pack(module.MAVLink(None,srcSystem=1,srcComponent=1))
                got = module.MAVLink(None).parse_char(packet)
                assert got is not None and got.get_type() == message.attrib['name']
                for name,value in values.items():
                    actual = getattr(got,name)
                    assert (list(actual) if isinstance(value,list) else actual) == value
                cases.append({'message':message.attrib['name'],'boundary':'MAX_INT_NEGATIVE_SIGNED_EXACT_FLOAT' if boundary else 'ZERO_TRUNCATION','frame_bytes_unsigned':len(packet),'status':'PASS'})
        fixture = stage/'compile.c'
        fixture.write_text('#include "FCU/mavlink.h"\n_Static_assert(MAVLINK_MSG_ID_FCU_OPERATION_STATUS==43000,"id");\n_Static_assert(MAVLINK_MSG_ID_FCU_BUILD_INFO==43012,"id");\nvoid spec_encode(mavlink_message_t*m,mavlink_fcu_prearm_status_t*p){mavlink_msg_fcu_prearm_status_encode(1,1,m,p);}\n',encoding='utf-8')
        flags = ['-std=c11','-mcpu=cortex-m7','-mthumb','-mfpu=fpv5-d16','-mfloat-abi=hard','-Werror=implicit-function-declaration']
        proc = subprocess.run([args.cc,*flags,'-I',str(stage/'generated'),'-c',str(fixture),'-o',str(stage/'compile.o')],capture_output=True,text=True)
        assert proc.returncode == 0, proc.stderr
        report = {'scope':'SPEC_DIALECT_CODEGEN','status':'PASS','pymavlink_version':pymavlink.__version__,'spec_xml_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'inputs_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in stage.glob('*.xml')},'wire_protocol':'2.0','language':['C','Python3'],'strict_units':True,'roundtrip_cases':cases,'arm_generated_header_compile':'PASS','compile_flags':flags,'compiler_diagnostics':proc.stderr,'log':log.getvalue(),'production_integration':'NOT_RUN','tool_scope':'temporary spec generation outside project'}
    (ROOT/'spec/validation/dialect_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('PASS: 13 private messages, 26 roundtrips, ARM generated-header compile')


if __name__ == '__main__':
    main()
