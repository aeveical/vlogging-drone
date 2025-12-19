import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { LoadingManager } from 'three';

const manager = new LoadingManager();
const loader = new GLTFLoader(manager);
let scrollY = 0;
let scrollProgress = 0;
let maxScroll = getMaxScroll();
window.addEventListener("scroll", () => {
    scrollY = window.scrollY;
    scrollProgress = Math.min(Math.max(scrollY/maxScroll, 0), 1);
});

let camera;
let mixer;
let action;
let droneTop;
let clip;
let targetTime = 0;
let currentTime = 0;
const smoothing = 0.08;
const panels = document.querySelectorAll(".pixhawk, .jetson, .camera, .vents, .landing-gear, .motors");

const canvas = document.getElementById("three");
const renderer = new THREE.WebGLRenderer({ canvas });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.setScissorTest(true);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x303030);

const light1 = new THREE.DirectionalLight(0xffffff, 3);
light1.position.set(3, 3, 1);
light1.intensity = 5;
scene.add(light1);

const light2 = new THREE.DirectionalLight(0xffffff, 3);
light2.position.set(-3, -3, -3);
light2.intensity = 5;
scene.add(light2);

initializeMesh('assets/drone-full-animation.glb').then((model) => {
    droneTop = model;
});

function animate() {
    requestAnimationFrame(animate);
    console.log("inside animate function");
    targetTime = scrollProgress * clip.duration;
    console.log(scrollProgress);
    // 0.7407407407407407
    // 0.8216468896080547

    panels.forEach(panel => {
        const start = parseFloat(panel.dataset.start);
        const end = parseFloat(panel.dataset.end);

        if (scrollProgress >= start && scrollProgress <= end) {
            panel.classList.add('active');
        } else {
            panel.classList.remove('active');
        }
    });

    currentTime = THREE.MathUtils.lerp(
        currentTime,
        targetTime,
        smoothing
    )

    action.time = currentTime;
    mixer.update(0);

    const width = window.innerWidth;
    const height = window.innerHeight;

    const scale = 1;
    const vpWidth = width * scale;
    const vpHeight = height * scale;

    const x = (width - vpWidth) / 2;
    const y = (height - vpHeight) / 2;

    renderer.setViewport(x, y, vpWidth, vpHeight);
    renderer.setScissor(x, y, vpWidth, vpHeight);
    
    renderer.render(scene, camera);
}

function getMaxScroll() {
    const documentHeight = document.body.scrollHeight;
    const viewportHeight = window.innerHeight;
    const maxScroll = documentHeight - viewportHeight;
    return Math.max(0, maxScroll);
}

function initializeMesh(modelAddress) {
    return new Promise((resolve, reject) => {
        loader.load(
            modelAddress,
            (gltf) => {
                const model = gltf.scene;
                camera = gltf.cameras[0];
                const animations = gltf.animations;
                mixer = new THREE.AnimationMixer(model);

                const box = new THREE.Box3().setFromObject(model);
                const center = new THREE.Vector3();
                box.getCenter(center);
                model.position.sub(center);

                camera.aspect = window.innerWidth/window.innerHeight;
                camera.updateProjectionMatrix();
                scene.add(model);

                if (animations && animations.length > 0) {
                    clip = animations[0];
                    action = mixer.clipAction(clip);
                    action.play();
                }

                resolve({model, mixer});
            },
            undefined,
            reject
        );
    });
}

manager.onLoad = () => {
    animate();
    console.log("triggered on load");
}
