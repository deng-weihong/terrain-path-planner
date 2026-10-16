import { existsSync } from 'node:fs'
import { delimiter, join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { spawn } from 'node:child_process'

const root = fileURLToPath(new URL('../', import.meta.url))
const venv = join(root, '.venv', process.platform === 'win32' ? 'Scripts/python.exe' : 'bin/python')
const python = process.env.PATH_PLANNER_PYTHON || (existsSync(venv) ? venv : 'python3')
const env = { ...process.env, PYTHONPATH: [join(root, 'src'), process.env.PYTHONPATH].filter(Boolean).join(delimiter) }
const child = spawn(python, process.argv.slice(2), { cwd: root, env, stdio: 'inherit' })
child.on('error', error => { console.error(`Cannot start ${python}: ${error.message}`); process.exitCode = 1 })
child.on('exit', code => { process.exitCode = code ?? 1 })
for (const signal of ['SIGINT', 'SIGTERM']) process.on(signal, () => child.kill(signal))
