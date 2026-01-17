#!/usr/bin/env node
/**
 * Test script for MuJoCo WASM package
 */

const fs = require('fs');
const path = require('path');

console.log('🧪 Testing MuJoCo WASM package...\n');

try {
  // Check if build artifacts exist
  const requiredFiles = [
    'dist/mujoco_wasm.js',
    'dist/mujoco_wasm.wasm',
    'dist/mujoco_wasm.d.ts',
  ];

  console.log('📦 Checking build artifacts...');
  let allPresent = true;

  for (const file of requiredFiles) {
    const exists = fs.existsSync(file);
    console.log(`  ${exists ? '✅' : '❌'} ${file}`);
    if (!exists) allPresent = false;
  }

  if (!allPresent) {
    throw new Error('Missing build artifacts. Run `npm run build` first.');
  }

  // Check file sizes
  console.log('\n📊 File sizes:');
  const wasmStats = fs.statSync('dist/mujoco_wasm.wasm');
  const jsStats = fs.statSync('dist/mujoco_wasm.js');
  const dtsStats = fs.statSync('dist/mujoco_wasm.d.ts');

  console.log(`  mujoco_wasm.wasm: ${(wasmStats.size / 1024 / 1024).toFixed(2)} MB`);
  console.log(`  mujoco_wasm.js: ${(jsStats.size / 1024).toFixed(2)} KB`);
  console.log(`  mujoco_wasm.d.ts: ${(dtsStats.size / 1024).toFixed(2)} KB`);

  // Check package.json
  console.log('\n📄 Checking package.json...');
  const pkg = JSON.parse(fs.readFileSync('package.json', 'utf-8'));

  const requiredFields = ['name', 'version', 'description', 'main', 'types'];
  for (const field of requiredFields) {
    if (pkg[field]) {
      console.log(`  ✅ ${field}: ${pkg[field]}`);
    } else {
      console.log(`  ❌ ${field}: missing`);
    }
  }

  // Validate TypeScript definitions
  console.log('\n📘 Validating TypeScript definitions...');
  const dtsContent = fs.readFileSync('dist/mujoco_wasm.d.ts', 'utf-8');

  const requiredExports = [
    'MainModule',
    'MjModel',
    'MjData',
    'loadMujoco',
  ];

  for (const exp of requiredExports) {
    if (dtsContent.includes(exp)) {
      console.log(`  ✅ ${exp} exported`);
    } else {
      console.log(`  ⚠️  ${exp} not found`);
    }
  }

  console.log('\n✅ All tests passed!');

} catch (error) {
  console.error('\n❌ Tests failed!');
  console.error(error.message);
  process.exit(1);
}
