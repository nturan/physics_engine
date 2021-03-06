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
                                        this.velocity.x*this.mass, 
                                        this.velocity.y*this.mass, 
                                        this.velocity.z*this.mass,
                                        this.quaternion.x, 
                                        this.quaternion.y, 
                                        this.quaternion.z,
                                        this.quaternion.w,
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

    UpdatePositionalState(t, step, integrator){
        let positional_state = math.matrix([
            this.state_vector._data[0],
            this.state_vector._data[1],
            this.state_vector._data[2],
            this.state_vector._data[3],
            this.state_vector._data[4],
            this.state_vector._data[5], 0, 0, 0, 0, 0, 0, 0
        ]);
        positional_state = integrator(
            this.DifferentialPositionalState.bind(this),
            positional_state, t, step, []);

        this.state_vector = math.matrix([
            positional_state._data[0],
            positional_state._data[1],
            positional_state._data[2],
            positional_state._data[3],
            positional_state._data[4],
            positional_state._data[5],
            this.state_vector._data[6],
            this.state_vector._data[7],
            this.state_vector._data[8],
            this.state_vector._data[9],
            this.state_vector._data[10],
            this.state_vector._data[11],
            this.state_vector._data[12]]);
        this.position = {x: this.state_vector._data[0], 
                         y: this.state_vector._data[1], 
                         z: this.state_vector._data[2]};
        let vel = this.CalculateVelocity(this.state_vector);
        this.velocity = {x: vel[0], y: vel[1], z: vel[2]};
        this.force = {x: 0.0, y: 0.0, z: 0.0};
    }

    UpdateRotationalState(t, step, integrator){
        let rotational_state = math.matrix([
            0, 0, 0, 0, 0, 0,
            this.state_vector._data[6],
            this.state_vector._data[7],
            this.state_vector._data[8],
            this.state_vector._data[9],
            this.state_vector._data[10],
            this.state_vector._data[11],
            this.state_vector._data[12]
        ]);

        rotational_state = integrator(
            this.DifferentialRotationalState.bind(this),
            rotational_state, t, step, []);

        this.state_vector = math.matrix([
            this.state_vector._data[0],
            this.state_vector._data[1],
            this.state_vector._data[2],
            this.state_vector._data[3],
            this.state_vector._data[4],
            this.state_vector._data[5],
            rotational_state._data[6],
            rotational_state._data[7],
            rotational_state._data[8],
            rotational_state._data[9],
            rotational_state._data[10],
            rotational_state._data[11],
            rotational_state._data[12]]);

        let newQ = new THREE.Quaternion(this.state_vector._data[6], 
                                        this.state_vector._data[7],
                                        this.state_vector._data[8], 
                                        this.state_vector._data[9]);
        newQ.normalize();
    
        this.quaternion = newQ;
        this.state_vector.subset(math.index([6, 7, 8, 9]), [newQ.x, 
                                                            newQ.y, 
                                                            newQ.z, 
                                                            newQ.w]);
        
        this.euler.setFromQuaternion(newQ);
        let wQ = this.CalculateW(this.state_vector);
        this.w = {x: wQ.x, y: wQ.y, z: wQ.z};
        this.L = math.matrix([[this.state_vector._data[10]], 
                              [this.state_vector._data[11]], 
                              [this.state_vector._data[12]]]);
        

        this.torque = {x: 0.0, y: 0.0, z: 0.0};
    }

    UpdateStateVector(t, step, integrator){
        this.UpdatePositionalState(t, step, integrator);
        this.UpdateRotationalState(t, step, integrator);
    }

    DifferentialPositionalState(t, x, arg){
        let F = this.force;
        let v = this.CalculateVelocity(x); // return as array
        let res = math.matrix([v[0], v[1], v[2], //velocity
                            F.x, F.y, F.z, // linear momentum
                            0, 0, 0, 0, 0, 0, 0
                           ]);
        return res;
    }

    DifferentialRotationalState(t, x, arg){
        let tau = this.torque;
        let w = this.CalculateW(x); //return as three.js quaternion
        // let w = this.w;
    
        let q = new THREE.Quaternion(x._data[6], x._data[7],
                                     x._data[8], x._data[9]);
        
        q.normalize();
        let qs = new THREE.Quaternion();
        qs.multiplyQuaternions(q, w);
    //    qs.normalize();
    
        let res = math.matrix([0, 0, 0, 0, 0, 0, 1/2*qs.x, 1/2*qs.y, 1/2*qs.z, 1/2*qs.w, tau.x, tau.y, tau.z ]);
        return res;
    }

    CalculateVelocity(x){
        return [x._data[3]/this.mass,
                x._data[4]/this.mass,
                x._data[5]/this.mass]
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
};

let Integrator = {

    euler: function (fun, x, t, step, arg) {
      let k1 = fun(t, x, arg);
      return math.add(x, math.multiply(step, k1));
    },

    rk4: function (fun, x, t, step, arg) {
      
      let k1 = fun(t, x, arg);
      let k2 = fun(t+step/2, math.add(x, math.multiply(step/2, k1)), arg);
      let k3 = fun(t+step/2, math.add(x, math.multiply(step/2, k2)), arg);
      let k4 = fun(t+step,   math.add(x, math.multiply(step,   k3)), arg);
      
      return math.add(x, math.multiply(step/6, math.add(k1,
                                                        math.multiply(2, k2),
                                                        math.multiply(2, k3),
                                                        k4)));
  
    }
  
};

export {PhysicsBody, Integrator};