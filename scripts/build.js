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

  // Build MuJoCo WASM using official CMake build
  console.log('\n🔨 Building MuJoCo WASM using official CMake...');
  console.log('This may take several minutes...\n');

  // Create build directory
  const buildDir = path.join(mujocoDir, 'build');
  if (fs.existsSync(buildDir)) {
    fs.rmSync(buildDir, { recursive: true, force: true });
  }
  fs.mkdirSync(buildDir, { recursive: true });

  // Configure with CMake using emcmake wrapper
  console.log('📋 Configuring with CMake (emcmake)...');
  const cmakeArgs = [
    'emcmake',
    'cmake',
    '..',
    '-DCMAKE_BUILD_TYPE=Release',
    '-DCMAKE_CXX_STANDARD=20',
    '-DCMAKE_CXX_STANDARD_REQUIRED=ON',
    '-DMUJOCO_BUILD_WASM=ON',
    '-DMUJOCO_BUILD_TESTS=OFF',
    '-DMUJOCO_SIMULATE=OFF',
    '-DMUJOCO_PYTHON=OFF',
    '-DMUJOCO_UI=OFF',
  ].join(' ');

  execSync(cmakeArgs, {
    stdio: 'inherit',
    cwd: buildDir,
  });

  // Build
  console.log('\n🔧 Building with CMake...');
  execSync('cmake --build . --parallel', {
    stdio: 'inherit',
    cwd: buildDir,
  });

  // Copy WASM files from MuJoCo's official output location
  console.log('\n📦 Copying WASM files...');
  const wasmDist = path.join(mujocoDir, 'wasm', 'dist');

  if (!fs.existsSync(wasmDist)) {
    throw new Error(`MuJoCo WASM build output not found at: ${wasmDist}`);
  }

  const files = ['mujoco_wasm.js', 'mujoco_wasm.wasm'];
  for (const file of files) {
    const src = path.join(wasmDist, file);
    const dest = path.join('dist', file);
    if (!fs.existsSync(src)) {
      throw new Error(`Expected file not found: ${src}`);
    }
    fs.copyFileSync(src, dest);
    console.log(`  ✅ ${file}`);
  }

  // Copy TypeScript definitions if they exist
  const dtsFile = 'mujoco_wasm.d.ts';
  const dtsSrc = path.join(wasmDist, dtsFile);
  if (fs.existsSync(dtsSrc)) {
    fs.copyFileSync(dtsSrc, path.join('dist', dtsFile));
    console.log(`  ✅ ${dtsFile}`);
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
