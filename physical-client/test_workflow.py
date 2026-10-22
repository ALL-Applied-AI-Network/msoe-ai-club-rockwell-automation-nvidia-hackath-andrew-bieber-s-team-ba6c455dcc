"""Offline shell/restart regressions. Fake uv never contacts any remote service."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys
import unittest

ROOT=Path(__file__).resolve().parent


class WorkflowTests(unittest.TestCase):
    def test_gpu_preflight_rejects_cpu_only_sam_environment(self):
        with tempfile.TemporaryDirectory() as tmp:
            modules=Path(tmp)
            (modules/'torch.py').write_text('class cuda:\n    @staticmethod\n    def is_available(): return False\n')
            (modules/'sam2.py').write_text('')
            env={**os.environ,'SORT_SAM_PYTHON':sys.executable,'PYTHONPATH':str(modules)}
            result=subprocess.run([sys.executable,str(ROOT/'preflight.py'),'--sam','--require-cuda'],
                                  env=env,text=True,capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('CUDA unavailable',result.stderr)

    def test_sam_preflight_reports_missing_external_interpreter(self):
        with tempfile.TemporaryDirectory() as tmp:
            env={**os.environ,'SORT_SAM_PYTHON':str(Path(tmp)/'not-installed')}
            result=subprocess.run([sys.executable,str(ROOT/'preflight.py'),'--sam'],
                                  env=env,text=True,capture_output=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Missing SAM interpreter',result.stderr)

    def sandbox(self, tmp):
        root=Path(tmp)
        for name in ('setup_rosie.sh','request_test.sh'):
            shutil.copy2(ROOT/name,root/name)
        (root/'submission').mkdir()
        (root/'submission/placement.yaml').write_text('rollouts: []\n')
        (root/'bin').mkdir()
        uv=root/'bin/uv'
        uv.write_text('#!/usr/bin/env bash\n'
                      'printf "%s\\n" "$*" >> "$WORKFLOW_LOG"\n'
                      'if [[ -e "$WORKFLOW_FAIL" && "$1" == sync ]]; then exit 9; fi\n')
        uv.chmod(0o755)
        env={**os.environ,'HOME':str(root/'home'),'PATH':str(root/'bin')+os.pathsep+os.environ['PATH'],
             'WORKFLOW_LOG':str(root/'calls.log'),'WORKFLOW_FAIL':str(root/'interrupted')}
        return root,env

    def test_request_generation_does_not_submit_or_connect(self):
        with tempfile.TemporaryDirectory() as tmp:
            root,env=self.sandbox(tmp)
            private=root/'.request.json';private.write_text('private-existing-request')
            proc=subprocess.run(['bash',str(root/'request_test.sh')],input='test@localhost\n',
                                text=True,capture_output=True,env=env,check=True)
            calls=(root/'calls.log').read_text().splitlines()
            self.assertEqual(len(calls),2)
            self.assertTrue(calls[0].startswith('run prepare_submission.py --contact Andrew Bieber, test@localhost'))
            self.assertEqual(calls[1],'run preflight.py --submission')
            self.assertIn('Nothing has been submitted',proc.stdout)
            self.assertEqual(private.read_text(),'private-existing-request')

    def test_setup_stops_at_failed_sync_and_can_restart(self):
        with tempfile.TemporaryDirectory() as tmp:
            root,env=self.sandbox(tmp)
            (root/'interrupted').write_text('simulate interrupted install')
            private=root/'.request.json';private.write_text('keep-private')
            failed=subprocess.run(['bash',str(root/'setup_rosie.sh')],env=env,capture_output=True)
            self.assertEqual(failed.returncode,9)
            self.assertEqual((root/'calls.log').read_text().splitlines(),['--version','sync --locked'])
            (root/'interrupted').rename(root/'previous-interruption')
            subprocess.run(['bash',str(root/'setup_rosie.sh')],env=env,capture_output=True,check=True)
            calls=(root/'calls.log').read_text().splitlines()[2:]
            self.assertEqual(calls[:2],['--version','sync --locked'])
            self.assertTrue(any('unittest' in c for c in calls))
            self.assertEqual(calls[-1],'run run.py --mock --trials 1 --trial-s 3')
            self.assertFalse(any('--submit' in c or '--status' in c for c in calls))
            self.assertEqual(private.read_text(),'keep-private')

    def test_private_uv_bypasses_broken_path_uv(self):
        with tempfile.TemporaryDirectory() as tmp:
            root,env=self.sandbox(tmp)
            private_uv=root/'home/.local/physical-uv/bin/uv'
            private_uv.parent.mkdir(parents=True)
            shutil.copy2(root/'bin/uv',private_uv)
            (root/'bin/uv').write_text('#!/usr/bin/env bash\necho broken-snap >&2\nexit 97\n')
            subprocess.run(['bash',str(root/'setup_rosie.sh')],env=env,capture_output=True,check=True)
            calls=(root/'calls.log').read_text().splitlines()
            self.assertIn('sync --locked',calls)
            self.assertIn('run run.py --mock --trials 1 --trial-s 3',calls)

    def test_snap_uv_is_replaced_and_restart_preserves_request(self):
        with tempfile.TemporaryDirectory() as tmp:
            root,env=self.sandbox(tmp)
            (root/'bin/uv').write_text('#!/usr/bin/env bash\necho broken-snap >&2\nexit 97\n')
            # Fake readlink identifies the discovered snap even on hosts without snap.
            readlink=root/'bin/readlink'
            readlink.write_text('#!/usr/bin/env bash\necho /usr/bin/snap\n')
            readlink.chmod(0o755)
            private_bin=root/'home/.local/physical-uv/bin'
            bootstrap=root/'bin/python3'
            bootstrap.write_text('#!/usr/bin/env bash\n'
                'mkdir -p "$HOME/.local/physical-uv/bin"\n'
                'cp "$BOOTSTRAP_UV" "$HOME/.local/physical-uv/bin/uv"\n'
                'printf "#!/usr/bin/env bash\\nexit 0\\n" > "$HOME/.local/physical-uv/bin/python"\n'
                'chmod +x "$HOME/.local/physical-uv/bin/python"\n')
            bootstrap.chmod(0o755)
            fixture=root/'uv-fixture'
            fixture.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$*" >> "$WORKFLOW_LOG"\n')
            fixture.chmod(0o755)
            env['BOOTSTRAP_UV']=str(fixture)
            private=root/'.request.json';private.write_text('existing-request')
            for _ in range(2):
                subprocess.run(['bash',str(root/'setup_rosie.sh')],env=env,capture_output=True,check=True)
            self.assertTrue((private_bin/'uv').is_file())
            self.assertEqual(private.read_text(),'existing-request')
            self.assertEqual((root/'calls.log').read_text().splitlines().count('sync --locked'),2)


if __name__=='__main__':
    unittest.main()
