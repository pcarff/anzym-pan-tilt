/**
 * visualizer.js
 * 
 * 3D Pan-Tilt Gimbal Visualizer using Three.js.
 * Renders real-time Azimuth (Pan) and Elevation (Tilt) orientation with target indicators.
 */

class GimbalVisualizer {
    constructor(canvasId, containerId) {
        this.canvas = document.getElementById(canvasId);
        this.container = document.getElementById(containerId);
        
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;

        // Visual 3D objects
        this.panGroup = null;     // Rotates around Y axis (Pan / Azimuth)
        this.tiltGroup = null;    // Rotates around X axis (Tilt / Elevation)
        this.targetLaser = null;
        this.ghostLaser = null;

        // Orientation angles in degrees
        this.currentPan = 0;
        this.currentTilt = 0;
        this.targetPan = 0;
        this.targetTilt = 0;

        this.initialized = false;
    }

    init() {
        if (!window.THREE) {
            console.warn("Three.js not loaded, falling back to 2D canvas.");
            this.init2DFallback();
            return;
        }

        try {
            const width = this.container.clientWidth || 400;
            const height = this.container.clientHeight || 300;

            // Scene setup
            this.scene = new THREE.Scene();
            this.scene.background = new THREE.Color(0x0c111d);

            // Camera setup
            this.camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
            this.camera.position.set(3.5, 2.8, 4.2);

            // WebGL Renderer
            this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true });
            this.renderer.setSize(width, height);
            this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
            this.renderer.shadowMap.enabled = true;

            // Orbit Controls
            if (window.THREE.OrbitControls) {
                this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
                this.controls.enableDamping = true;
                this.controls.dampingFactor = 0.05;
                this.controls.maxPolarAngle = Math.PI / 2 + 0.2; // Don't go far below floor
                this.controls.minDistance = 2.0;
                this.controls.maxDistance = 10.0;
                this.controls.target.set(0, 1.2, 0);
            }

            // Lighting
            const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
            this.scene.add(ambientLight);

            const dirLight = new THREE.DirectionalLight(0x38bdf8, 1.2);
            dirLight.position.set(5, 10, 7);
            this.scene.add(dirLight);

            const pointLight = new THREE.PointLight(0x10b981, 0.8, 10);
            pointLight.position.set(-3, 3, -3);
            this.scene.add(pointLight);

            // Build Scene Objects
            this.buildEnvironment();
            this.buildGimbalModel();

            // Handle Resizing
            window.addEventListener('resize', () => this.onWindowResize());

            this.initialized = true;
            this.animate();
        } catch (e) {
            console.error("Error initializing 3D WebGL:", e);
            this.init2DFallback();
        }
    }

    buildEnvironment() {
        // Grid floor
        const grid = new THREE.GridHelper(8, 16, 0x0ea5e9, 0x1e293b);
        grid.position.y = 0;
        this.scene.add(grid);

        // Cardinal Direction Compass Ring
        const ringGeo = new THREE.RingGeometry(1.8, 1.85, 32);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x334155, side: THREE.DoubleSide });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        ring.rotation.x = Math.PI / 2;
        ring.position.y = 0.01;
        this.scene.add(ring);

        // North Pointer (Red marker pointing -Z / North)
        const arrowGeo = new THREE.ConeGeometry(0.12, 0.35, 16);
        const arrowMat = new THREE.MeshBasicMaterial({ color: 0xef4444 });
        const arrow = new THREE.Mesh(arrowGeo, arrowMat);
        arrow.position.set(0, 0.02, -2.1);
        arrow.rotation.x = -Math.PI / 2;
        this.scene.add(arrow);
    }

    buildGimbalModel() {
        const matBase = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.4, metalness: 0.8 });
        const matPanTurntable = new THREE.MeshStandardMaterial({ color: 0x0369a1, roughness: 0.3, metalness: 0.7 });
        const matFork = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.4, metalness: 0.6 });
        const matTiltPayload = new THREE.MeshStandardMaterial({ color: 0x10b981, roughness: 0.3, metalness: 0.7 });
        const matLens = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.1, metalness: 0.9 });
        const matLaser = new THREE.MeshBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.85 });

        // 1. Base Mount Hierarchy
        this.baseGroup = new THREE.Group();
        this.scene.add(this.baseGroup);

        // 2. Pan Group (Rotates around Y-axis with Pan/Azimuth)
        this.panGroup = new THREE.Group();
        this.panGroup.position.y = 0;
        this.scene.add(this.panGroup);

        // 3. Tilt Group (Pivot at elevation axle center Y=1.40, rotates around X-axis)
        this.tiltGroup = new THREE.Group();
        this.tiltGroup.position.set(0, 1.40, 0);
        this.panGroup.add(this.tiltGroup);

        // Target Line / Beam projecting forward along -Z direction
        const laserGeo = new THREE.CylinderGeometry(0.015, 0.015, 6.0, 8);
        const laser = new THREE.Mesh(laserGeo, matLaser);
        laser.position.set(0, 0, -3.5);
        laser.rotation.x = Math.PI / 2;
        this.tiltGroup.add(laser);
        this.targetLaser = laser;

        // Target Reticle Circle at laser tip
        const reticleGeo = new THREE.RingGeometry(0.15, 0.18, 24);
        const reticleMat = new THREE.MeshBasicMaterial({ color: 0x38bdf8, side: THREE.DoubleSide });
        const reticle = new THREE.Mesh(reticleGeo, reticleMat);
        reticle.position.set(0, 0, -6.5);
        this.tiltGroup.add(reticle);

        // --- Build Placeholder / Fallback Geometries ---
        this.placeholderBase = new THREE.Mesh(new THREE.CylinderGeometry(0.84, 0.84, 0.2, 32), matBase);
        this.placeholderBase.position.y = 0.1;
        this.baseGroup.add(this.placeholderBase);

        this.placeholderPan = new THREE.Group();
        const panDiscMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.8, 0.8, 0.15, 32), matPanTurntable);
        panDiscMesh.position.y = 0.25;
        this.placeholderPan.add(panDiscMesh);

        const armGeo = new THREE.BoxGeometry(0.18, 1.1, 0.3);
        const armLeft = new THREE.Mesh(armGeo, matFork);
        armLeft.position.set(-0.55, 0.85, 0);
        this.placeholderPan.add(armLeft);

        const armRight = new THREE.Mesh(armGeo, matFork);
        armRight.position.set(0.55, 0.85, 0);
        this.placeholderPan.add(armRight);
        this.panGroup.add(this.placeholderPan);

        this.placeholderTilt = new THREE.Group();
        const axleMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.08, 0.08, 1.2, 16), matBase);
        axleMesh.rotation.z = Math.PI / 2;
        this.placeholderTilt.add(axleMesh);

        const bodyMesh = new THREE.Mesh(new THREE.BoxGeometry(0.65, 0.45, 0.85), matTiltPayload);
        this.placeholderTilt.add(bodyMesh);

        const barrelMesh = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.2, 0.4, 32), matLens);
        barrelMesh.rotation.x = Math.PI / 2;
        barrelMesh.position.set(0, 0, -0.55);
        this.placeholderTilt.add(barrelMesh);
        this.tiltGroup.add(this.placeholderTilt);

        // --- Asynchronously Load Real CAD STL Models ---
        this.loadSTLModels();
    }

    loadSTLModels() {
        if (!window.THREE || !window.THREE.STLLoader) {
            console.log("STLLoader not ready, using geometric visualizer.");
            return;
        }

        const loader = new THREE.STLLoader();
        const matBase = new THREE.MeshStandardMaterial({ 
            color: 0x1e293b, 
            roughness: 0.35, 
            metalness: 0.85 
        });
        const matTurretTop = new THREE.MeshStandardMaterial({ 
            color: 0x0284c7, 
            roughness: 0.3, 
            metalness: 0.75 
        });
        const matShaft = new THREE.MeshStandardMaterial({ 
            color: 0x10b981, 
            roughness: 0.3, 
            metalness: 0.7 
        });
        const matGears = new THREE.MeshStandardMaterial({ 
            color: 0xf59e0b, 
            roughness: 0.25, 
            metalness: 0.85 
        });

        // 1. Turret Base (Stationary ground mount)
        loader.load('models/Turret_base.stl', (geometry) => {
            geometry.computeVertexNormals();
            geometry.rotateX(-Math.PI / 2);
            geometry.rotateY(-Math.PI / 2); // 90° clockwise rotation for 0-direction
            geometry.scale(0.01, 0.01, 0.01);
            const mesh = new THREE.Mesh(geometry, matBase);
            mesh.position.set(0, 0.1, 0); // Align bottom to Y=0
            mesh.castShadow = true;
            mesh.receiveShadow = true;

            if (this.placeholderBase) {
                this.baseGroup.remove(this.placeholderBase);
                this.placeholderBase = null;
            }
            this.baseGroup.add(mesh);
            console.log("✓ Loaded Turret_base.stl CAD model");
        }, undefined, (e) => console.warn("Notice: Using procedural base geometry"));

        // 2. Turret Top (Pan rotating fork)
        loader.load('models/Turret_top.stl', (geometry) => {
            geometry.computeVertexNormals();
            geometry.rotateX(-Math.PI / 2);
            geometry.rotateY(-Math.PI / 2); // 90° clockwise rotation for 0-direction
            geometry.scale(0.01, 0.01, 0.01);
            const mesh = new THREE.Mesh(geometry, matTurretTop);
            mesh.position.set(0, 0.1, 0);
            mesh.castShadow = true;
            mesh.receiveShadow = true;

            if (this.placeholderPan) {
                this.panGroup.remove(this.placeholderPan);
                this.placeholderPan = null;
            }
            this.panGroup.add(mesh);
            console.log("✓ Loaded Turret_top.stl CAD model");
        }, undefined, (e) => console.warn("Notice: Using procedural pan fork geometry"));

        // 3. Turret Shaft (Tilt elevation cradle & cross-mount platform)
        loader.load('models/Turret_Shaft.stl', (geometry) => {
            geometry.computeVertexNormals();
            geometry.center(); // Center on elevation pivot axis
            geometry.rotateY(Math.PI / 2); // Spans horizontally across left & right arms
            geometry.rotateX(Math.PI);     // Flip right-side up along the axle
            geometry.scale(0.01, 0.01, 0.01);
            const mesh = new THREE.Mesh(geometry, matShaft);
            mesh.castShadow = true;
            mesh.receiveShadow = true;

            if (this.placeholderTilt) {
                this.tiltGroup.remove(this.placeholderTilt);
                this.placeholderTilt = null;
            }
            this.tiltGroup.add(mesh);
            console.log("✓ Loaded Turret_Shaft.stl CAD model");
        }, undefined, (e) => console.warn("Notice: Using procedural tilt cradle geometry"));

        // 4. Stepper Drive Gears & Bushings Accents
        loader.load('models/Turret_stepper_gears.stl', (geometry) => {
            geometry.computeVertexNormals();
            geometry.rotateX(-Math.PI / 2);
            geometry.rotateY(-Math.PI / 2);
            geometry.scale(0.01, 0.01, 0.01);
            const mesh = new THREE.Mesh(geometry, matGears);
            mesh.position.set(0.1, 0.1, -1.0);
            this.panGroup.add(mesh);
            console.log("✓ Loaded Turret_stepper_gears.stl");
        });
    }

    updateAngles(panDeg, tiltDeg, targetPanDeg, targetTiltDeg) {
        this.currentPan = panDeg;
        this.currentTilt = tiltDeg;
        this.targetPan = (targetPanDeg !== undefined) ? targetPanDeg : panDeg;
        this.targetTilt = (targetTiltDeg !== undefined) ? targetTiltDeg : tiltDeg;

        if (this.panGroup && this.tiltGroup) {
            // Convert Pan & Tilt to Three.js Euler rotations
            // Pan: Azimuth around Y axis (North = 0 deg / -Z, East = +90 deg / +X)
            // In Three.js: -pan in radians gives standard clockwise azimuth
            this.panGroup.rotation.y = -THREE.MathUtils.degToRad(this.currentPan);

            // Tilt: Altitude around X axis (0 deg = horizon / forward, +90 = Zenith / up)
            this.tiltGroup.rotation.x = THREE.MathUtils.degToRad(this.currentTilt);
        }
    }

    resetCamera() {
        if (this.camera && this.controls) {
            this.camera.position.set(3.5, 2.8, 4.2);
            this.controls.target.set(0, 1.2, 0);
            this.controls.update();
        }
    }

    onWindowResize() {
        if (!this.renderer || !this.camera || !this.container) return;
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    animate() {
        if (!this.initialized) return;
        requestAnimationFrame(() => this.animate());

        if (this.controls) {
            this.controls.update();
        }

        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
        }
    }

    init2DFallback() {
        // Fallback 2D canvas drawing if WebGL is unavailable
        const ctx = this.canvas.getContext('2d');
        if (!ctx) return;

        const draw2D = () => {
            ctx.fillStyle = '#0c111d';
            ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);

            const cx = this.canvas.width / 2;
            const cy = this.canvas.height / 2;
            const r = Math.min(cx, cy) - 20;

            // Pan Dial
            ctx.strokeStyle = '#0ea5e9';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, 2 * Math.PI);
            ctx.stroke();

            // Heading needle
            const rad = (this.currentPan - 90) * (Math.PI / 180);
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 4;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + Math.cos(rad) * (r - 10), cy + Math.sin(rad) * (r - 10));
            ctx.stroke();

            requestAnimationFrame(draw2D);
        };
        draw2D();
    }
}

window.GimbalVisualizer = GimbalVisualizer;

