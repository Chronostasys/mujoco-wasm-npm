# MuJoCo WASM

MuJoCo physics engine compiled to WebAssembly with JavaScript/TypeScript bindings.

This package provides the MuJoCo physics engine (Multi-Joint dynamics with Contact) for use in web browsers via WebAssembly. MuJoCo is a high-performance physics simulator widely used in robotics, biomechanics, and reinforcement learning.

## Features

- ✅ Full MuJoCo physics engine in WebAssembly
- ✅ TypeScript type definitions included
- ✅ No native dependencies - pure JavaScript + WASM
- ✅ High performance (near-native speed)
- ✅ Compatible with modern browsers (Chrome, Firefox, Safari, Edge)
- ✅ Support for all MuJoCo features:
  - Rigid body dynamics
  - Collision detection
  - Friction and contact
  - Actuators and sensors
  - Rendering (optional)
  - XML model loading

## Installation

```bash
npm install @your-org/mujoco-wasm
```

## Quick Start

```javascript
import loadMujoco from '@your-org/mujoco-wasm';

// Load MuJoCo WASM
const mujoco = await loadMujoco();

// Load a model from XML
const xmlContent = `
<mujoco model="test">
  <worldbody>
    <geom name="floor" type="plane" size="100 100 1"/>
    <body name="box" pos="0 0 0.5">
      <geom name="box" type="box" size="0.1 0.1 0.1"/>
      <joint type="free"/>
    </body>
  </worldbody>
</mujoco>
`;

// Write XML to virtual filesystem
mujoco.FS.writeFile('/tmp/model.xml', xmlContent);

// Load model
const model = mujoco.MjModel.mj_loadXML('/tmp/model.xml');
const data = new mujoco.MjData(model);

// Step simulation
for (let i = 0; i < 1000; i++) {
  mujoco.mj_step(model, data);
}

// Access state
const qpos = data.qpos;  // Joint positions
const qvel = data.qvel;  // Joint velocities
```

## Documentation

### API Reference

#### `loadMujoco(): Promise<MuJoCoModule>`

Asynchronously loads the MuJoCo WASM module.

```typescript
interface MuJoCoModule {
  // Model and Data
  MjModel: {
    mj_loadXML(filename: string): MjModel;
    mj_delete(model: MjModel): void;
    // ... more methods
  };

  MjData: {
    constructor(model: MjModel): MjData;
    // ... properties
  };

  // Simulation
  mj_step(model: MjModel, data: MjData): void;
  mj_forward(model: MjModel, data: MjData): void;

  // Virtual filesystem
  FS: {
    writeFile(path: string, data: string): void;
    readFile(path: string): string;
    unlink(path: string): void;
  };

  // Utilities
  mju_malloc(size: number): number;
  mju_free(ptr: number): void;
}
```

### Model XML Format

MuJoCo models are defined in XML format. See [MuJoCo Documentation](https://mujoco.readthedocs.io/) for details.

Basic structure:

```xml
<mujoco model="my_model">
  <compiler angle="radian" coordinate="local"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>

  <worldbody>
    <!-- Ground -->
    <geom name="floor" type="plane" size="100 100 1"/>

    <!-- Bodies -->
    <body name="object" pos="0 0 1">
      <geom name="geom" type="box" size="0.1 0.1 0.1"/>
      <joint name="joint" type="free"/>
    </body>
  </worldbody>

  <actuator>
    <motor joint="joint" gear="100"/>
  </actuator>
</mujoco>
```

## Browser Support

Tested on modern browsers with WebAssembly support:

- Chrome 57+
- Firefox 52+
- Safari 11+
- Edge 16+

## Performance

- **WASM file size**: ~7.5 MB (uncompressed)
- **Load time**: ~1-3 seconds on modern connections
- **Simulation speed**: 1000-10000 steps/second (depending on model complexity)
- **Memory usage**: ~50-200 MB (depending on model)

## Development

### Building from Source

If you want to build the WASM files yourself:

```bash
# Install Emscripten
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install latest
./emsdk activate latest
source ./emsdk_env.sh

# Clone MuJoCo
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco

# Build WASM (see scripts/build.js for details)
npm run build
```

### Using Docker

```bash
npm run build:docker
```

## Acknowledgments

- [MuJoCo](https://github.com/google-deepmind/mujoco) by DeepMind
- [Emscripten](https://emscripten.org/) for C++ to WASM compilation

## License

Apache License 2.0

See [LICENSE](LICENSE) file for details.

## Support

- GitHub Issues: https://github.com/your-org/mujoco-wasm/issues
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- MuJoCo Forum: https://mujoco.org/forum/
