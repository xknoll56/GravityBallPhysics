#pragma once

enum GBCardinal
{
    PosX = 0, NegX,
    PosY, NegY,
    PosZ, NegZ,
    None
};


struct GBTransform {
    GBVector3 position;
    GBQuaternion rotation;


    // Constructors
    GBTransform()
        : position(0, 0, 0), rotation() {
    }

    GBTransform(const GBVector3& pos, const GBQuaternion& rot)
        : position(pos), rotation(rot) {
    }


    // Combine two transforms: this * other
    GBTransform operator*(const GBTransform& other) const {
        GBTransform result;
        result.position = position + rotation * (other.position);
        result.rotation = rotation * other.rotation;
        result.rotation.normalize();
        return result;
    }

    // Setters with dirty flag
    void setPosition(const GBVector3& pos)
    {
        if (!position.epsilonEqual(pos))
        {
            position = pos;
        }
    }

    void translate(const GBVector3& delta)
    {
        position += delta;
	}

    void setRotation(const GBQuaternion& rot)
    {
        // You could compare with epsilon if needed
        rotation = rot;
        rotation.normalize();
    }

    void rotate(const GBQuaternion& deltaRot)
    {
        rotation = deltaRot * rotation;
        rotation.normalize();
	}

    // Inverse transform
    GBTransform inverse() const {
        GBQuaternion invRot = rotation.conjugate();
        GBVector3 invPos = invRot * -(position);
        return GBTransform(invPos, invRot);
    }

    GBMatrix toMatrix() const {
        GBMatrix R = rotation.toMatrix();   // 3×3 rotation
        GBMatrix M;

        // Scale the basis vectors (columns)
        M.m[0][0] = R.m[0][0];
        M.m[1][0] = R.m[1][0];
        M.m[2][0] = R.m[2][0];
                             
        M.m[0][1] = R.m[0][1];
        M.m[1][1] = R.m[1][1];
        M.m[2][1] = R.m[2][1];
                             
        M.m[0][2] = R.m[0][2];
        M.m[1][2] = R.m[1][2];
        M.m[2][2] = R.m[2][2];

        // Translation (4th column)
        M.m[0][3] = position.x;
        M.m[1][3] = position.y;
        M.m[2][3] = position.z;

        return M;
    }



    // Transform a point from local space to world space
    GBVector3 transformPoint(const GBVector3& localPoint) const {
        return rotation * (localPoint) + position;
    }

    // Transform a direction (ignores translation, scales)
    GBVector3 transformDirection(const GBVector3& dir) const {
        return rotation * (dir);
    }

    // Look at a target (does not affect scale)
    void lookAt(const GBVector3& target, const GBVector3& up = { 0, 0, 1 }) {
        rotation = GBQuaternion::lookAt(position, target, up);
    }

    GBVector3 forward() const { return rotation * GBVector3(1, 0, 0); } // X
    GBVector3 right() const { return rotation * GBVector3(0, 1, 0); }   // Y
    GBVector3 up() const { return rotation * GBVector3(0, 0, 1); }      // Z

    GBVector3 worldToLocalPoint(const GBVector3& worldPoint) const {
        GBVector3 p = worldPoint - position;     // remove translation
        return rotation.conjugate() * p;                  // apply inverse rotation
    }

    // Convert local-space point to world space
    GBVector3 localToWorldPoint(const GBVector3& localPoint) const {
        return transformPoint(localPoint);
    }

    // Convert world-space direction to local space
    GBVector3 worldToLocalDir(const GBVector3& worldDir) const {
        return  rotation.conjugate() * worldDir; // rotate by inverse
    }

    // Convert local-space direction to world space
    GBVector3 localToWorldDir(const GBVector3& localDir, bool normalize = true) const {
        GBVector3 dir = transformDirection(localDir);
        if (normalize) {
            return dir.normalized();
        }
        return dir;
    }

    // Local-space rotation (Unreal-style)
    void rotateLocal(const GBQuaternion& deltaRot) {
        rotation = rotation * deltaRot;
        rotation.normalize();
    }
};

struct GBFrame
{
    GBVector3 forward;
    GBVector3 right;
    GBVector3 up;

    GBFrame(GBVector3 forward, GBVector3 right, GBVector3 up):
        right(right), up(up), forward(forward)
    {

    }

    GBFrame(const GBTransform& transform)
    {
        right = transform.right();
        up = transform.up();
        forward = transform.forward();
    }

    GBFrame()
    {
        right = GBVector3::right();
        up = GBVector3::up();
        forward = GBVector3::forward();
    }

    void applyTransform(const GBTransform& transform)
    {
        right = transform.transformDirection(right);
        up = transform.transformDirection(up);
        forward = transform.transformDirection(forward);
    }

    GBQuaternion toRotation()
    {
       return GBQuaternion::fromAxes(
            forward,
            right,
            up
        );
    }
};
