package frc.robot.subsystems.swerve;

// Explanation Provided by Sukesh Kumar
// Main Programmer : Ritesh Raj

public class RiteshManFilter {
    private final double[][] A;  // Process model
    private final double[][] H;  // Measurement model
    private final double[][] Q;  // Process noise covariance
    private final double[][] R;  // Measurement noise covariance
    private double[][] P;  // Error covariance matrix
    private double[][] x;  // State vector

    public RiteshManFilter(double dt) {
        // Time step (I would probably do every 1/10 of a second)

        // State transition matrix (A) (This matrix basically telling kalman for every timestep the behavior of how the angle and angular velocity is changing)
        //This code is perfect leave it
        A = new double[][] {
                {1 , dt} , //The top row is for angles, 1 means for every 1/10 (or whatever time step) that the angle is changing at a constant rate (linearly)
                {0 , 1}  //This row is for angular velocity 1 means for every 1/10 (or whatever time step) that the angular velocity is changing at a constant rate (linearly).
        }; //Im not sure what the 0 means but like everything else makes sense sooooooo

        // Measurement matrix (H) (This matrix is basically telling Kalman what it should expect from the pigeon)
        H = new double[][] {
                {1 , 0} , //1 means the angle measurement, 0 means ur saying that the angle is exactly what it is from the pigeon
                {0 , 1} //Same thing here but for angular velocity
        };

        // Process noise covariance (Q) (This matrix is saying how much difference this filter can differ from actual pigeon readings)
        //THIS IS THE STUFF TO CHANGE!!!
        Q = new double[][] {
                {0.001 , 0} , //0.001 is the angle variance that you're expecting that the kalman filter can be off by. 0 means this measurement is only related to the pigeon
                {0 , 0.003} //0.003 is the angular velocity variance for the same stuff
        };

        // Measurement noise covariance (R) (This matrix is saying how noisy/dumb the pigeon is to kalman)
        //THIS IS THE STUFF TO CHANGE!!!
        R = new double[][] {
                {0.01 , 0} , //0.01 is how dumb were expecting the pigeons angle sensing to be. 0 means that this variance is directly related to the pigeon
                {0 , 0.1} //0.1 is for the angular velocity, same stuff above
        };

        // Initial covariance estimate (P) (This basically means how much u trust about the initial values that were given to the kalman filter)
        P = new double[][] {
                {1 , 0} , //Im pretty sure its on a scale of 0 < x < 2 (in this models case), 0 means u completely trust the intial angle data being given to the filter, 2 means you dont trust it at all.
                {0 , 1}  //Same thing here but for angular velocity
        };

        // Initial state estimate (angle, angular velocity)
        x = new double[][] {
                {0} ,  // initial angle
                {0}   // initial angular velocity
        };
    }

    public void predict(double u_angle , double u_angular_velocity) {
        // Control input vector (u)
        double[][] u = new double[][] {
                {u_angle} ,
                {u_angular_velocity}
        };

        // Predict state: x = A * x + B * u
        x = matrixAdd(matrixMultiply(A , x) , u);

        // Predict covariance: P = A * P * A^T + Q
        P = matrixAdd(matrixMultiply(matrixMultiply(A , P) , transpose(A)) , Q);
    }

    public void update(double measured_angle , double measured_angular_velocity) {
        // Measurement vector (z)
        double[][] z = new double[][] {
                {measured_angle} ,
                {measured_angular_velocity}
        };

        // Compute Kalman Gain: K = P * H^T * (H * P * H^T + R)^-1
        double[][] S = matrixAdd(matrixMultiply(matrixMultiply(H , P) , transpose(H)) , R);
        double[][] K = matrixMultiply(P , transpose(H));
        K = matrixMultiply(K , inverse(S));

        // Update state: x = x + K * (z - H * x)
        double[][] y = matrixSubtract(z , matrixMultiply(H , x));
        x = matrixAdd(x , matrixMultiply(K , y));

        // Update covariance: P = (I - K * H) * P
        double[][] I = identityMatrix(2);
        P = matrixMultiply(matrixSubtract(I , matrixMultiply(K , H)) , P);
    }

    public double getAngle() {
        return x[ 0 ][ 0 ];
    }

    public double getAngularVelocity() {
        return x[ 1 ][ 0 ];
    }

    private double[][] matrixAdd(double[][] a , double[][] b) {
        int rows = a.length;
        int cols = a[ 0 ].length;
        double[][] result = new double[ rows ][ cols ];
        for ( int i = 0 ; i < rows ; i++ ) {
            for ( int j = 0 ; j < cols ; j++ ) {
                result[ i ][ j ] = a[ i ][ j ] + b[ i ][ j ];
            }
        }
        return result;
    }

    private double[][] matrixMultiply(double[][] a , double[][] b) {
        int rowsA = a.length;
        int colsA = a[ 0 ].length;
        int colsB = b[ 0 ].length;
        double[][] result = new double[ rowsA ][ colsB ];
        for ( int i = 0 ; i < rowsA ; i++ ) {
            for ( int j = 0 ; j < colsB ; j++ ) {
                for ( int k = 0 ; k < colsA ; k++ ) {
                    result[ i ][ j ] += a[ i ][ k ] * b[ k ][ j ];
                }
            }
        }
        return result;
    }

    private double[][] matrixSubtract(double[][] a , double[][] b) {
        int rows = a.length;
        int cols = a[ 0 ].length;
        double[][] result = new double[ rows ][ cols ];
        for ( int i = 0 ; i < rows ; i++ ) {
            for ( int j = 0 ; j < cols ; j++ ) {
                result[ i ][ j ] = a[ i ][ j ] - b[ i ][ j ];
            }
        }
        return result;
    }

    private double[][] transpose(double[][] a) {
        int rows = a.length;
        int cols = a[ 0 ].length;
        double[][] result = new double[ cols ][ rows ];
        for ( int i = 0 ; i < rows ; i++ ) {
            for ( int j = 0 ; j < cols ; j++ ) {
                result[ j ][ i ] = a[ i ][ j ];
            }
        }
        return result;
    }

    private double[][] inverse(double[][] a) {
        int n = a.length;
        double[][] x = new double[ n ][ n ];
        double[][] b = new double[ n ][ n ];
        int[] index = new int[ n ];
        for ( int i = 0 ; i < n ; ++i ) {
            b[ i ][ i ] = 1;
        }
        gaussian(a , index);
        for ( int i = 0 ; i < n - 1 ; ++i ) {
            for ( int j = i + 1 ; j < n ; ++j ) {
                for ( int k = 0 ; k < n ; ++k ) {
                    b[ index[ j ] ][ k ] -= a[ index[ j ] ][ i ] * b[ index[ i ] ][ k ];
                }
            }
        }
        for ( int i = 0 ; i < n ; ++i ) {
            x[ n - 1 ][ i ] = b[ index[ n - 1 ] ][ i ] / a[ index[ n - 1 ] ][ n - 1 ];
            for ( int j = n - 2 ; j >= 0 ; --j ) {
                x[ j ][ i ] = b[ index[ j ] ][ i ];
                for ( int k = j + 1 ; k < n ; ++k ) {
                    x[ j ][ i ] -= a[ index[ j ] ][ k ] * x[ k ][ i ];
                }
                x[ j ][ i ] /= a[ index[ j ] ][ j ];
            }
        }
        return x;
    }

    private void gaussian(double[][] a , int[] index) {
        int n = index.length;
        double[] c = new double[ n ];
        for ( int i = 0 ; i < n ; ++i ) {
            index[ i ] = i;
        }
        for ( int i = 0 ; i < n ; ++i ) {
            double c1 = 0;
            for ( int j = 0 ; j < n ; ++j ) {
                double c0 = Math.abs(a[ i ][ j ]);
                if ( c0 > c1 ) {
                    c1 = c0;
                }
            }
            c[ i ] = c1;
        }
        int k = 0;
        for ( int j = 0 ; j < n - 1 ; ++j ) {
            double pi1 = 0;
            for ( int i = j ; i < n ; ++i ) {
                double pi0 = Math.abs(a[ index[ i ] ][ j ]);
                pi0 /= c[ index[ i ] ];
                if ( pi0 > pi1 ) {
                    pi1 = pi0;
                    k = i;
                }
            }
            int itmp = index[ j ];
            index[ j ] = index[ k ];
            index[ k ] = itmp;
            for ( int i = j + 1 ; i < n ; ++i ) {
                double pj = a[ index[ i ] ][ j ] / a[ index[ j ] ][ j ];
                a[ index[ i ] ][ j ] = pj;
                for ( int l = j + 1 ; l < n ; ++l ) {
                    a[ index[ i ] ][ l ] -= pj * a[ index[ j ] ][ l ];
                }
            }
        }
    }

    private double[][] identityMatrix(int size) {
        double[][] identity = new double[ size ][ size ];
        for ( int i = 0 ; i < size ; i++ ) {
            identity[ i ][ i ] = 1;
        }
        return identity;
    }
}