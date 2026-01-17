#!/usr/bin/env node
/**
 * MuJoCo WASM Build Script
 *
 * This script builds MuJoCo to WebAssembly using Emscripten.
 * It can run locally or in CI/CD environments.
 */

const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

// Determine MuJoCo version to use
// If triggered by a tag in CI, use the tag version; otherwise use 'main' branch
function getMujocoVersion() {
  // Check if we're in a CI environment with a tag trigger
  const githubRef = process.env.GITHUB_REF || '';
  const githubRefName = process.env.GITHUB_REF_NAME || '';
  
  // If triggered by a tag (format: refs/tags/v2.3.8 or v2.3.8)
  if (githubRef.startsWith('refs/tags/')) {
    let tagVersion = githubRef.replace('refs/tags/', '');
    // Remove 'v' prefix if present (v2.3.8 -> 2.3.8)
    if (tagVersion.startsWith('v')) {
      tagVersion = tagVersion.substring(1);
    }
    console.log(`📌 Detected tag trigger: ${githubRef}, using MuJoCo version: ${tagVersion}`);
    return tagVersion;
  } else if (githubRefName && githubRefName.startsWith('v')) {
    let tagVersion = githubRefName;
    // Remove 'v' prefix if present
    if (tagVersion.startsWith('v')) {
      tagVersion = tagVersion.substring(1);
    }
    console.log(`📌 Detected tag from GITHUB_REF_NAME: ${githubRefName}, using MuJoCo version: ${tagVersion}`);
    return tagVersion;
  }
  
  // Default: use 'main' branch for latest version
  return 'main';
}

const MUJOCO_VERSION = getMujocoVersion();
const EMSCRIPTEN_FLAGS = [
  '-O3',                              // Optimization level
  '-s WASM=1',                        // Generate WebAssembly
  '-s ALLOW_MEMORY_GROWTH=1',         // Allow memory to grow
  '-s MODULARIZE=1',                  // Create ES6 module
  '-s EXPORT_NAME="\'MuJoCo\'"',      // Export name
  '-s EXPORT_ES6=1',                  // Export ES6 module
  '-s USE_ES6_IMPORT_META=0',         // Not using import.meta
  '-s NO_DYNAMIC_EXECUTION=1',        // No eval()
  '-s SINGLE_FILE=0',                 // Separate .wasm file
  '-s ASSERTIONS=0',                  // Disable assertions (release)
  '-s DISABLE_EXCEPTION_CATCHING=1',  // Disable exception catching
  '-fno-exceptions',                  // C++ no exceptions
  '-std=c++17',                       // C++17 standard
  '--bind',                           // Use embind
  '-o', 'dist/mujoco_wasm.js',        // Output file
];

console.log('🔨 MuJoCo WASM Build Script');
console.log('=========================\n');
console.log(`📦 Using MuJoCo version: ${MUJOCO_VERSION}\n`);

try {
  // Check if emscripten is available
  console.log('🔍 Checking Emscripten installation...');
  try {
    const emccVersion = execSync('emcc --version', { encoding: 'utf-8' });
    console.log('✅ Emscripten found:\n', emccVersion.split('\n')[0]);
  } catch (error) {
    console.error('❌ Emscripten not found!');
    console.error('Please install Emscripten:');
    console.error('  git clone https://github.com/emscripten-core/emsdk.git');
    console.error('  cd emsdk && ./emsdk install latest && ./emsdk activate latest');
    console.error('  source ./emsdk_env.sh');
    process.exit(1);
  }

  // Clean previous build
  console.log('\n🧹 Cleaning previous build...');
  if (fs.existsSync('dist')) {
    fs.rmSync('dist', { recursive: true, force: true });
  }
  fs.mkdirSync('dist', { recursive: true });

  // Clone MuJoCo if not present
  const mujocoDir = 'build/mujoco';
  if (!fs.existsSync(mujocoDir)) {
    console.log('\n📥 Cloning MuJoCo...');
    try {
      // First, try cloning with the specific version as tag/branch
      execSync(
        `git clone --depth 1 --branch ${MUJOCO_VERSION} https://github.com/google-deepmind/mujoco.git ${mujocoDir}`,
        { stdio: 'inherit' }
      );
    } catch (error) {
      // If that fails, clone main branch and checkout the tag
      console.log(`⚠️  Branch/tag ${MUJOCO_VERSION} not found, cloning main branch...`);
      execSync(
        `git clone --depth 1 https://github.com/google-deepmind/mujoco.git ${mujocoDir}`,
        { stdio: 'inherit' }
      );
      // Try to checkout the tag if it exists
      try {
        execSync(
          `cd ${mujocoDir} && git fetch --depth 1 origin tag ${MUJOCO_VERSION} 2>/dev/null && git checkout ${MUJOCO_VERSION}`,
          { stdio: 'inherit' }
        );
        console.log(`✅ Checked out tag ${MUJOCO_VERSION}`);
      } catch (tagError) {
        // If tag doesn't exist either, just use main branch
        console.log(`⚠️  Tag ${MUJOCO_VERSION} not found, using main branch`);
      }
    }
  } else {
    console.log('\n✅ MuJoCo source already present');
  }

  // Build MuJoCo WASM
  console.log('\n🔨 Building MuJoCo WASM...');
  console.log('This may take several minutes...\n');

  // Source files relative to mujocoDir
  const sourceFiles = [
    'src/user/user_api.cc',
    'src/user/user_objects.cc',
    'src/user/model.cc',
    'src/xml/xml_native.cc',
    'src/xml/xml_numeric.cc',
    'src/engine/engine_engine.cc',
    'src/engine/engine_io.cc',
    'src/engine/engine_solver.cc',
    'src/engine/engine_util_blas.cc',
    'src/engine/engine_util_misc.cc',
    'src/engine/engine_util_solve.cc',
    'src/engine/engine_util_sparse.cc',
    'src/engine/engine_callback.cc',
    'src/engine/engine_plugin.cc',
    'src/user/user_composite.cc',
  ];

  // Include directories relative to mujocoDir
  const includeDirs = [
    'include',
    'src',
    'dist',
  ].map(dir => `-I${dir}`);

  const buildCommand = [
    'emcc',
    ...sourceFiles,
    ...includeDirs,
    ...EMSCRIPTEN_FLAGS,
  ].join(' ');

  console.log('Build command:');
  console.log(buildCommand.substring(0, 200) + '...\n');

  // Execute from mujocoDir so all relative paths work correctly
  execSync(buildCommand, {
    stdio: 'inherit',
    cwd: mujocoDir,
  });

  // Move files to dist directory
  console.log('\n📦 Packaging files...');
  const buildDist = path.join(mujocoDir, 'dist');
  const files = ['mujoco_wasm.js', 'mujoco_wasm.wasm'];

  for (const file of files) {
    const src = path.join(buildDist, file);
    const dest = path.join('dist', file);
    fs.copyFileSync(src, dest);
    console.log(`  ✅ ${file}`);
  }

  // Copy TypeScript definitions
  const dtsSource = path.join(mujocoDir, 'wasm', 'dist', 'mujoco_wasm.d.ts');
  if (fs.existsSync(dtsSource)) {
    fs.copyFileSync(dtsSource, 'dist/mujoco_wasm.d.ts');
    console.log('  ✅ mujoco_wasm.d.ts');
  }

  // Generate .npm package info
  console.log('\n📊 Build summary:');
  const stats = fs.statSync('dist/mujoco_wasm.wasm');
  console.log(`  WASM file size: ${(stats.size / 1024 / 1024).toFixed(2)} MB`);

  console.log('\n✅ Build completed successfully!');
  console.log('\nOutput files:');
  console.log('  📄 dist/mujoco_wasm.js');
  console.log('  📄 dist/mujoco_wasm.wasm');
  console.log('  📄 dist/mujoco_wasm.d.ts');

} catch (error) {
  console.error('\n❌ Build failed!');
  console.error(error.message);
  if (error.stdout) console.error('STDOUT:', error.stdout);
  if (error.stderr) console.error('STDERR:', error.stderr);
  process.exit(1);
}
