// Tools that work on the user's own Horizon OS firmware: OTA payload extraction, filesystem
// listing and ELF compatibility analysis. Pure JVM so it runs in tests and on desktop too.
plugins {
    id("org.jetbrains.kotlin.jvm")
    application
}

application {
    mainClass.set("org.metaport.firmware.MainKt")
    applicationName = "metaport-firmware"
    applicationDefaultJvmArgs = listOf("-Dstdout.encoding=UTF-8", "-Dfile.encoding=UTF-8")
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

kotlin {
    compilerOptions {
        jvmTarget.set(org.jetbrains.kotlin.gradle.dsl.JvmTarget.JVM_17)
    }
}

dependencies {
    implementation("org.tukaani:xz:1.10")                        // REPLACE_XZ operations
    implementation("org.apache.commons:commons-compress:1.27.1") // REPLACE_BZ operations
    testImplementation(kotlin("test"))
    testImplementation("org.junit.jupiter:junit-jupiter:5.11.3")
    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

tasks.test {
    useJUnitPlatform()
}
