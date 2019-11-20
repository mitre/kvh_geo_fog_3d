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
	    timeout(time: 2, unit: 'HOURS') 
    }
    agent
    {
        label 'docker-ubuntu-1804'
    }
    triggers
    {
        //Trigger builds every night at 2AM
        cron('H 2 * * *')
    }
    stages
    {
        stage('Checkout')
        {
            steps
            {
                CheckoutMelodicDevel()
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
                sh script: """#!/bin/bash
                    sudo -E apt-get update --fix-missing
                """, label: 'Apt Cache Update'
                //Setup the OS, specifically for ROS Melodic
                SetupMelodic()
            }
        } //end: stage('Setup')
        stage('Pre-Build Code Analysis')
        {
            steps
            {
                script
                {
                    warnError('Catkin Linter Failed!')
                    {
                        CatkinLint()
                    }
                    warnError('CppCheck Failed!')
                    {
                        CppCheck()
                    }
                    warnError('Lizard Complexity Analysis Failed!')
                    {
                        Lizard()
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
        stage('Post-Build Code Analysis')
        {
            steps
            {
                script
                {
                    warnError('ClangTidy Failed!')
                    {
                        ClangTidy()
                    }
                    warnError('RosLint Failed!')
                    {
                        RosLint()
                    }
                }
            }
        }
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
                //sh script: 'echo "Packaging disabled for now"', label: "Packaging"
            }
        }
    } //end: stages
    
    post
    {
        always
        {
            archiveArtifacts 'catkin_ws/*_lint.txt'
            archiveArtifacts 'catkin_ws/src/kvh_geo_fog_3d/cppcheck_output/*.cppcheck'
            archiveArtifacts 'lizard.xml'
            archiveArtifacts 'catkin_ws/build/kvh_geo_fog_3d_driver/test_results/kvh_geo_fog_3d_driver/gtest-kvh_geo_fog_3d_driver-test.xml'
            archiveArtifacts 'catkin_ws/src/kvh_geo_fog_3d/clang_format_kvh_geo_fog_3d.tar.gz'
            archiveArtifacts 'catkin_ws/src/kvh_geo_fog_3d/clangtidy/*.clangtidy'
            archiveArtifacts 'catkin_ws/src/kvh_geo_fog_3d/roslint_output/*.txt'
            
            ////////////////////////////////////////////////////////////////////
            // Due to how fragile plugin publishers are with Declarative
            // Pipelines in Jenkins right now, I'm wrapping all of the publisher
            // calls in warnError(). Once the plugin maintainers get their stuff
            // together, I will be smarter about this.
            ////////////////////////////////////////////////////////////////////
            script
            {
                //CPPCHECK
                warnError('Publishing CppCheck Results Failed!')
                {
                    //Using the warnings-ng plugin
                    recordIssues enabledForFailure: false, aggregatingResults : false, tool: cppCheck(pattern: 'catkin_ws/src/kvh_geo_fog_3d/cppcheck_output/*.cppcheck')
                }
                //LIZARD
                //warnError('Publishing Lizard Results Failed!')
                //{
                //    step([$class: 'hudson.plugins.cppncss.CppNCSSPublisher', reportFilenamePattern: 'lizard.xml', functionCcnViolationThreshold: 5, functionNcssViolationThreshold: 10, targets: []])
                //}
                //GCC Warnings/Errors
                warnError('Publishing GCC Warnings/Errors Failed!')
                {
                    recordIssues enabledForFailure: false, aggregatingResults : false, tool: gcc4()
                }
		//Clang-tidy
                warnError('Publishing Clang-Tidy Results Failed!')
                {
		    //Use warnings-ng to publish clangtidy
		    recordIssues(tools: [clangTidy(pattern: 'catkin_ws/src/kvh_geo_fog_3d/clangtidy/*.clangtidy')])
                }
                //Unit Testing
                warnError('Publishing Unit Test Results Failed!')
                {
                    xunit (thresholds: [ skipped(failureThreshold: '0'), failed(failureThreshold: '0') ],
                        tools: [ GoogleTest(pattern: 'catkin_ws/build/kvh_geo_fog_3d_driver/test_results/kvh_geo_fog_3d_driver/gtest-kvh_geo_fog_3d_driver-test.xml') ])
                }
            }
	}
//	success
//	{
//            archiveArtifacts 'catkin_ws/src/*.deb'
//    	}
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


void CheckoutMelodicDevel()
{
    //Clone the repo
    checkout([$class: 'GitSCM',
        branches: [[name: '*/melodic-devel']],
        doGenerateSubmoduleConfigurations: false,
        extensions: [[$class: 'RelativeTargetDirectory',
            relativeTargetDir: 'catkin_ws/src/kvh_geo_fog_3d']],
        submoduleCfg: [],
        userRemoteConfigs:
            [[credentialsId:'ZLAC_GITLAB_KEY',
            url: 'git@gitlab.mitre.org:DART/kvh_geo_fog_3d.git']]
        ])
}
void SetupMelodic()
{
    //Use rosdep to install dependencies for our package, as defined by its
    //package.xml. Assumes that the current user has nopasswd set for sudo,
    //since rosdep calls sudo for apt-get install.
    sh script: """
        cd catkin_ws
        rosdep update
        rosdep install --rosdistro melodic --from-paths src -y -r src/kvh_geo_fog_3d
    """, label: 'Install Package rosdeps'
}
void BuildRelease()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d
    sh script: '''#!/bin/bash
        cd catkin_ws
        source /opt/ros/melodic/setup.bash
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCATKIN_ENABLE_TESTING=1''', label: 'Build Release'
}
void BuildTest()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d
    sh script: '''#!/bin/bash
        cd catkin_ws
        source /opt/ros/melodic/setup.bash
        catkin build --verbose --catkin-make-args run_tests''', label: 'Build Test'
}
void BuildDebug()
{
    //Initial directory is ${JENKINS_WORKSPACE}/kvh_geo_fog_3d.
    //Under here is contained catkin_ws/src/kvh_geo_fog_3d
    sh script: '''#!/bin/bash
        cd catkin_ws
        set -o pipefail
        source /opt/ros/melodic/setup.bash
        catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_VERBOSE_MAKEFILE=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCATKIN_ENABLE_TESTING=1''', label: 'Build Debug'
}
void CatkinLint()
{
    sh script: '''#!/bin/bash
        cd catkin_ws
        set -o pipefail
        source /opt/ros/melodic/setup.bash
        catkin lint -W2 src/kvh_geo_fog_3d |& tee catkinpackage_lint.txt
    ''', label: 'Catkin Linter'
}
void CppCheck()
{
    //Run cppcheck
    sh script: """
        # Run this from the base of the workspace! File paths are relative to
        # the run location, and Jenkins reports want those paths to be relative
        # to the workspace root (e.g. /home/jenkins/workspace/kvh_geo_fog_3d_driver)
        cd catkin_ws/src/kvh_geo_fog_3d
        ./devops/cppcheck.sh kvh_geo_fog_3d
    """, label: 'CPPCheck'
}
void Lizard()
{
    sh script: """
        lizard -l cpp catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_driver/src/ catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_driver/include/ catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_rviz/src/ catkin_ws/src/kvh_geo_fog_3d/kvh_geo_fog_3d_rviz/include/ --xml > lizard.xml 2>&1
    """, label: 'Lizard'
}
void ClangTidy()
{
    sh script: """#!/bin/bash
        cd catkin_ws/src/kvh_geo_fog_3d
        source ../../devel/setup.bash
        ./devops/clang_tidy.sh kvh_geo_fog_3d
    """, label: 'Clang-tidy'
}
void RosLint()
{
    sh script: """#!/bin/bash
        cd catkin_ws/src/kvh_geo_fog_3d
        source ../../devel/setup.bash
        ./devops/roslint.sh
    """, label: 'Roslint'
}
void PackageDebian()
{
    sh script: '''#!/bin/bash
        echo "Currently building debs is not supported! Can't handle missing deps"
    ''', label: "Debian Packaging"
//    sh script: '''#!/bin/bash
//        cd catkin_ws/src/kvh_geo_fog_3d
//        ./build_all_debs.sh
//    ''', label: "Debian Packaging"
}

void SendEmail()
{
    //emailext body: "${currentBuild.currentResult}: Job ${env.JOB_NAME} build ${env.BUILD_NUMBER}\n More info at: ${env.BUILD_URL}",
    //            recipientProviders: [[$class: 'DevelopersRecipientProvider'], [$class: 'RequesterRecipientProvider']],
    //            subject: "Jenkins Build ${currentBuild.currentResult}: Job ${env.JOB_NAME}", to: 'zlacelle, tbostic'
}
