import * as THREE from 'https://unpkg.com/three/build/three.module.js';

class PhysicsBody {
    constructor(mass, position, velocity, 
        InertiaTensorCalculator = PhysicsBody.DefaultInertiaTensorCalculator) {
        this.mass = mass;
        this.position = position;
        this.velocity = velocity;
        this.CalculateInertiaTensor = InertiaTensorCalculator;
        this.inertia_tensor = this.CalculateInertiaTensor();
        this.force = {x: 0.0, y: 0.0, z: 0.0};
        this.torque = {x: 0.0, y: 0.0, z: 0.0};
        this.acceleration = {x: 0.0, y: 0.0, z: 0.0};
        this.quaternion = new THREE.Quaternion();
        this.w = {x: 0.0, y: 0.0, z: 0.0};
        this.L = math.multiply(this.inertia_tensor, 
            [[this.w.x],
            [this.w.y],
            [this.w.z]]);
        this.euler = new THREE.Euler();
        this.euler.setFromQuaternion(this.quaternion);
        this.state_vector = math.matrix([this.position.x, 
                                        this.position.y, 
                                        this.position.z,
                                        this.quaternion.x, 
                                        this.quaternion.y, 
                                        this.quaternion.z,
                                        this.quaternion.w,
                                        this.velocity.x*this.mass, 
                                        this.velocity.y*this.mass, 
                                        this.velocity.z*this.mass,
                                        this.L._data[0][0], 
                                        this.L._data[1][0], 
                                        this.L._data[2][0]
                                    ]);
    }

    ApplyForce(force){
        this.force.x += force.x;
        this.force.y += force.y;
        this.force.z += force.z;
    }

    ApplyTorque(torque){
        this.torque.x += torque.x;
        this.torque.y += torque.y;
        this.torque.z += torque.z;
    }

    UpdateStateVector(t, step){
        //let arg = [];
        this.state_vector = Integrator.rk4(this.dxdt.bind(this), this.state_vector, t, step, []);
    
        let newQ = new THREE.Quaternion(this.state_vector._data[3], 
                                        this.state_vector._data[4],
                                        this.state_vector._data[5], 
                                        this.state_vector._data[6]);
        newQ.normalize();
    
        this.quaternion = newQ;
        this.state_vector.subset(math.index([3, 4, 5, 6]), [newQ.x, 
                                                           newQ.y, 
                                                           newQ.z, 
                                                           newQ.w]);
    
        this.position = {x: this.state_vector._data[0], 
                         y: this.state_vector._data[1], 
                         z: this.state_vector._data[2]};
        let vel = this.CalculateVelocity(this.state_vector);
        this.velocity = {x: vel[0], y: vel[1], z: vel[2]};
        
        this.euler.setFromQuaternion(newQ);
        this.force = {x: 0, y: 0, z: 0};
        let wQ = this.CalculateW(this.state_vector);
        this.w = {x: wQ.x, y: wQ.y, z: wQ.z};
        this.L = math.matrix([[this.state_vector._data[10]], 
                              [this.state_vector._data[11]], 
                              [this.state_vector._data[12]]]);
        
        this.force = {x: 0.0, y: 0.0, z: 0.0};
        this.torque = {x: 0.0, y: 0.0, z: 0.0};
    }

    dxdt(t, x, arg){
        let F = this.force;
        let tau = this.torque;
        let v = this.CalculateVelocity(x); // return as array
        let w = this.CalculateW(x); //return as three.js quaternion
    
    
        let q = new THREE.Quaternion(x._data[3], x._data[4],
                                     x._data[5], x._data[6]);
    
    
    
    
        q.normalize();
        let qs = new THREE.Quaternion();
        qs.multiplyQuaternions(q, w);
    //    qs.normalize();
        let res = math.matrix([v[0], v[1], v[2], //velocity
                            1/2*qs.x, 1/2*qs.y, 1/2*qs.z, 1/2*qs.w, //rotation
                            F.x, F.y, F.z, // linear momentum
                            tau.x, tau.y, tau.z //angular momentum
                           ]);
        return res;
    }

    CalculateVelocity(x){
        return [x._data[7]/this.mass,
                x._data[8]/this.mass,
                x._data[9]/this.mass]
    }

    CalculateW(x){
        this.inertia_tensor = this.CalculateInertiaTensor();
        let w = math.multiply(math.inv(this.inertia_tensor), 
                              [[x._data[10]], 
                               [x._data[11]], 
                               [x._data[12]]]);
        return new THREE.Quaternion(w._data[0][0], w._data[1][0], w._data[2][0], 0);
    }


    static DefaultInertiaTensorCalculator(){
        return math.matrix([
            [1.0, 0.0, 0.0], 
            [0.0, 1.0, 0.0], 
            [0.0, 0.0, 1.0]])
    }
}

export {PhysicsBody};