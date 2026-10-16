import { spawnSync } from 'node:child_process'
import { fileURLToPath } from 'node:url'

const cwd = fileURLToPath(new URL('../', import.meta.url))
for (const args of [
  ['-S', 'cpp', '-B', 'build/cpp', '-DCMAKE_BUILD_TYPE=Release', ...process.argv.slice(2)],
  ['--build', 'build/cpp', '--parallel', process.env.CMAKE_BUILD_PARALLEL_LEVEL || '2'],
]) {
  const result = spawnSync('cmake', args, { cwd, stdio: 'inherit' })
  if (result.error) { console.error(result.error.message); process.exit(1) }
  if (result.status !== 0) process.exit(result.status ?? 1)
}
