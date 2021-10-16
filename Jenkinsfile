pipeline {
  agent {
    dockerfile {
      additionalBuildArgs '--build-arg USER_ID=$(id -u)'
    }
  }
  stages {
    stage('Build') {
      steps {
       sh '''#!/bin/bash -l
         mkdir build
         cd build
         cmake .. -DENABLE_TESTS=ON
         make -j8
       '''
      }
    }
    stage('Test') {
      steps {
           echo 'Testing..'
           sh '''#!/bin/bash -l
              cd build
              make test
           '''
      }
    }
  }
  post {
    cleanup {
      cleanWs()
    }
  }
}
