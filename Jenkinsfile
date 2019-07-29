////////////////////////////////////////////////////////////////////////////////
//                               NOTES
// This build works assuming the following:
//  * Your code is a catkin package
//  * Your package.xml is correct, and thus will install your required
//    dependencies using rosdep.
//  * Your package passes the catkin linter (will thrown a warning if it fails)
////////////////////////////////////////////////////////////////////////////////

pipeline
{
    options
    {
        // Keep N most recent builds
        buildDiscarder(logRotator(numToKeepStr:'20'))
    }
    agent
    {
        label 'docker-ubuntu-1604'
    }
    triggers
    {
        //Trigger builds every night at 2AM, as well as whenever the repo is
        //updated. Poll for updates every 10 minutes.
        pollSCM('H/10 * * * *')
        cron('H 2 * * *')
    }
    
    stages
    {
        stage('Checkout')
        {
            steps
            {
                CheckoutMaster()
            }
        } //end: stage('Checkout')
        stage('Setup')
        {
            steps
            {
                //Add useful SSH keys to known_hosts
                sh script: """
                    mkdir -p ~/.ssh
                    chmod 700 ~/.ssh
                    ssh-keyscan -H github.com >> ~/.ssh/known_hosts
                    ssh-keyscan -H gitlab.mitre.org >> ~/.ssh/known_hosts
                """, label: 'Setup github/gitlab SSH keys'
                //Setup the OS, specifically for ROS Kinetic
                SetupKinetic()
            }
        } //end: stage('Setup')
        stage('Code Analysis')
        {
            steps
            {
                script
                {
                    try
                    {
                        CatkinLint()
                    }
                    catch(exc)
                    {
                        currentBuild.result = 'UNSTABLE'
                    }
                    try
                    {
                        CppCheck()
                    }
                    catch(exc)
                    {
                        currentBuild.result = 'UNSTABLE'
                    }
                    try
                    {
                        Lizard()
                    }
                    catch(exc)
                    {
                        currentBuild.result = 'UNSTABLE'
                    }
                }
            }
        } //end: stage('Code Analysis')
        stage('Build')
        {
            steps
            {
                BuildRelease()
            }
        } //end: stage('Build')
        stage('Test')
        {
            steps
            {
                BuildTest()
            }
        } //end: stage('Test')
        stage('Package')
        {
            steps
            {
                PackageDebian()
            }
        }
    } //end: stages
    
    post
    {
        always
        {
            archiveArtifacts 'catkin_ws/*_lint.txt'
            archiveArtifacts 'cppcheck-result.xml'
            archiveArtifacts 'lizard.xml'
            archiveArtifacts 'catkin_ws/build/kvh_geo_fog_3d_driver/test_results/kvh_geo_fog_3d_driver/gtest-kvh_geo_fog_3d_driver-test.xml'
            
            ////////////////////////////////////////////////////////////////////
            // Due to how fragile plugin publishers are with Declarative
            // Pipelines in Jenkins right now, I'm wrapping all of the publisher
            // calls in try/catch. Once the plugin maintainers get their stuff
            // together, I will be smarter about this.
            ////////////////////////////////////////////////////////////////////
            script
            {
                //CPPCHECK
                try
                {
                    //Using the warnings-ng plugin
                    recordIssues enabledForFailure: false, aggregatingResults : false, tool: cppCheck(pattern: 'cppcheck-result.xml')
                }
                catch(exc)
                {
                    currentBuild.result = 'UNSTABLE'
                }
                //LIZARD
                try
                {
                    step([$class: 'hudson.plugins.cppncss.CppNCSSPublisher', reportFilenamePattern: 'lizard.xml', functionCcnViolationThreshold: 5, functionNcssViolationThreshold: 10, targets: []])
                }
                catch(exc)
                {
                    sh script: 'echo THIS IS BROKEN FOR NOW!!!', label: 'Lizard Publisher'
                }
                //GCC Warnings/Errors
                try
                {
                    recordIssues enabledForFailure: false, aggregatingResults : false, tool: gcc4()
                }
                catch(exc)
                {
                    currentBuild.result = 'UNSTABLE'
                }
            }
        }
	    success
	    {
            archiveArtifacts 'catkin_ws/src/*.deb'
    	}
        failure
        {
            SendEmail()
        }
        fixed
        {
            SendEmail()
        }
    }
} //end: pipeline


void CheckoutMaster()
{
    //Clone the repo
    checkout([$class: 'GitSCM',
        branches: [[name: '*/master']],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'RelativeTargetDirectory',
            relativeTargetDir: 'catkin_ws/src/kvh_geo_fog_3d_driver']],
        submoduleCfg: [],
        userRemoteConfigs:
            [[credentialsId:'ZLAC_GITLAB_KEY',
            url: 'git@gitlab.mitre.org:DART/kvh_geo_fog_3d_driver.git']]
        ])
}
void SetupKinetic()
{
    sh script: """
        sudo sh -c 'echo \"deb http://packages.ros.org/ros/ubuntu \$(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list'
        sudo sh -c 'apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'
        sudo apt-get update
        #Install "base" ROS kinetic packages. Includes basic build tools, the
        #ros-base meta-package, rosdep, and catkin.
        sudo apt-get -y install build-essential ros-kinetic-ros-base \
            python-rosdep python-catkin-tools python-catkin-lint python-bloom \
            fakeroot dpkg-dev debhelper
        """, label: 'Setup ROS Repository'
    //Initialize rosdep. Must be called with sudo.
    sh script: "sudo rosdep init", label: "rosdep init"
    //Use rosdep to install dependencies for our package, as defined by its
    //package.xml. Assumes that the current user has nopasswd set for sudo,
    //since rosdep calls sudo for apt-get install.
    sh script: """
        cd catkin_ws
        rosdep update
        rosdep install --rosdistro kinetic --from-paths src -y -r src/kvh_geo_fog_3d_driver
    """, label: 'Install Package rosdeps'
}
void BuildRelease()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d_driver.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d_driver
    sh script: '''#!/bin/bash
        cd catkin_ws
        source /opt/ros/kinetic/setup.bash
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_VERBOSE_MAKEFILE=ON''', label: 'Build Release'
}
void BuildTest()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d_driver.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d_driver
    sh script: '''#!/bin/bash
        cd catkin_ws
        source /opt/ros/kinetic/setup.bash
        catkin build --verbose --catkin-make-args run_tests''', label: 'Build Test'
}
void BuildDebug()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d_driver.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d_driver
    sh script: '''#!/bin/bash
        cd catkin_ws
        set -o pipefail
        source /opt/ros/kinetic/setup.bash
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_VERBOSE_MAKEFILE=ON''', label: 'Build Debug'
}
void CatkinLint()
{
    sh script: '''#!/bin/bash
        cd catkin_ws
        set -o pipefail
        source /opt/ros/kinetic/setup.bash
        catkin lint -W2 src/kvh_geo_fog_3d_driver |& tee catkinpackage_lint.txt
    ''', label: 'Catkin Linter'
}
void CppCheck()
{
    //Run cppcheck
    sh script: """
        # Run this from the base of the workspace! File paths are relative to
        # the run location, and Jenkins reports want those paths to be relative
        # to the workspace root (e.g. /home/jenkins/workspace/kvh_geo_fog_3d_driver)
        cppcheck --enable=warning,style,performance,portability --language=c++ --platform=unix64 --std=c++11 -I catkin_ws/src/kvh_geo_fog_3d_driver/include/ --xml --xml-version=2 catkin_ws/src/kvh_geo_fog_3d_driver/src 2> cppcheck-result.xml
    """, label: 'CPPCheck'
}
void Lizard()
{
    sh script: """
        lizard -l cpp catkin_ws/src/kvh_geo_fog_3d_driver/src/ catkin_ws/src/kvh_geo_fog_3d_driver/include/ --xml > lizard.xml 2>&1
    """, label: 'Lizard'
}
void PackageDebian()
{
    sh script: '''#!/bin/bash
        cd catkin_ws/src/kvh_geo_fog_3d_driver
        bloom-generate rosdebian --os-name ubuntu --os-version xenial --ros-distro kinetic
        cp packaging/rules debian/rules
        source /opt/ros/kinetic/setup.bash
        fakeroot debian/rules binary
    ''', label: "Debian Packaging"
}

void SendEmail()
{
    emailext body: "${currentBuild.currentResult}: Job ${env.JOB_NAME} build ${env.BUILD_NUMBER}\n More info at: ${env.BUILD_URL}",
                recipientProviders: [[$class: 'DevelopersRecipientProvider'], [$class: 'RequesterRecipientProvider']],
                subject: "Jenkins Build ${currentBuild.currentResult}: Job ${env.JOB_NAME}", to: 'zlacelle, tbostic'
}