# Nori
Assignments for Nori.

### Assignment 1 [Completed]

### Assignment 2 

#### Part 1: Octree construction [Completed]

#### Part 2: Ray traversal [Completed]

#### Part 3: Improved ray traversal [Completed]

#### Part 4: Efficiency [Completed]

#### Hacker Points: Parallelization 

### Assignment 3

#### Part 1: Monte Carlo Sampling [Completed]

#### Part 2: Two simple rendering algorithms [Completed]

##### Part 2.1: Point lights [Completed]

Rendering result:

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa3/ajax-simple.png" width="400">
      <br /> 
      <b>Simple Ajax, 768 * 768, spp: 32</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa3/ajax-simple-ref.png" width="400">
      <br />
      <b>Simple Ajax, reference</b><br>
    </td>
  </tr>
</table>

##### Part 2.2: Ambient occlusion [Completed]

Rendering result:

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa3/ajax-ao.png" width="400">
      <br /> 
      <b>AO Ajax, 768 * 768, spp: 512</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa3/ajax-ao-ref.png" width="400">
      <br />
      <b>AO Ajax, reference</b><br>
    </td>
  </tr>
</table>

#### Before Assignment 4: Support for Multi-Mesh Scenes [Completed]

### Assignment 4

#### Part 1: Area lights [Completed]

#### Part 2: Distribution Ray Tracing [Completed]

Rendering result:

1. Cornell Box

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa4/cbox/cbox-distributed.png" width="400">
      <br /> 
      <b>Cornell Box, 800 * 600, spp: 512</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa4/cbox/cbox-distributed-ref.png" width="400">
      <br />
      <b>Cornell Box, reference</b><br>
    </td>
  </tr>
</table>

2. EPFL Logo

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa4/logo/logo-diffuse.png" width="400">
      <br /> 
      <b>EPFL Logo, 768 * 384, spp: 4096</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa4/logo/logo-diffuse-ref.png" width="400">
      <br />
      <b>EPFL Logo, reference</b><br>
    </td>
  </tr>
</table>

Test result:

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa4/tests/test-mesh.png" width="400">
      <br /> 
      <b>test-mesh</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa4/tests/test-mesh-furnace.png" width="400">
      <br />
      <b>test-mesh-furnace</b><br>
    </td>
  </tr>
</table>

#### Part 3: Dielectrics [Completed]

#### Part 4: Whitted-style ray tracing [Completed]

Rendering result:

1. Cornell Box

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa4/cbox/cbox-whitted.png" width="400">
      <br /> 
      <b>Cornell Box, 800 * 600, spp: 512</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa4/cbox/cbox-whitted-ref.png" width="400">
      <br />
      <b>Cornell Box, reference</b><br>
    </td>
  </tr>
</table>

2. EPFL Logo

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa4/logo/logo-dielectric.png" width="400">
      <br /> 
      <b>EPFL Logo, 768 * 384, spp: 4096</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa4/logo/logo-dielectric-ref.png" width="400">
      <br />
      <b>EPFL Logo, reference</b><br>
    </td>
  </tr>
</table>

#### Artist Points: Interesting scene 

#### Hacker Points: Specialized Light Source Sampling

### Assignment 5

#### Part 1: Microfacet BRDF [Completed]

Rendering result:

1. Rough Ajax

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/ajax/ajax-rough.png" width="400">
      <br /> 
      <b>Rough Ajax, 768 * 768, spp: 64</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/ajax/ajax-rough-ref.png" width="400">
      <br />
      <b>Rough Ajax, reference</b><br>
    </td>
  </tr>
</table>

2. Smooth Ajax

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/ajax/ajax-smooth.png" width="400">
      <br /> 
      <b>Smooth Ajax, 768 * 768, spp: 64</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/ajax/ajax-smooth-ref.png" width="400">
      <br />
      <b>Smooth Ajax, reference</b><br>
    </td>
  </tr>
</table>

Test result:

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/chi2-microfacet.png" width="400">
      <br /> 
      <b>chi2-microfacet</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/ttest-microfacet.png" width="400">
      <br />
      <b>ttest-microfacet</b><br>
    </td>
  </tr>
</table>

#### Part 2: Brute force path tracer [Completed]

Rendering result:

1. Cornell Box 

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/cbox/cbox_mats.png" width="400">
      <br /> 
      <b>Cornell Box, 800 * 600, spp: 512</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/cbox/cbox_mats_ref.png" width="400">
      <br />
      <b>Cornell Box, reference</b><br>
    </td>
  </tr>
</table>

2. Veach Scene

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/veach_mi/veach_mats.png" width="400">
      <br /> 
      <b>Veach Scene, 768 * 512, spp: 256</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/veach_mi/veach_mats_ref.png" width="400">
      <br />
      <b>Veach Scene, reference</b><br>
    </td>
  </tr>
</table>

3. Table Scene

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/table/table_mats.png" width="400">
      <br /> 
      <b>Table Scene, 800 * 600, spp: 128</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/table/table_mats_ref.png" width="400">
      <br />
      <b>Table Scene, reference</b><br>
    </td>
  </tr>
</table>

Test result:

1. test-direct
<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-6.png" width="400">
      <br /> 
      <b>Test 6</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-7.png" width="400">
      <br />
      <b>Test 7</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-8.png" width="400">
      <br />
      <b>Test 8</b><br>
    </td>
  </tr>
</table>

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-9.png" width="400">
      <br /> 
      <b>Test 9</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-10.png" width="400">
      <br />
      <b>Test 10</b><br>
    </td>
  </tr>
</table>

2. test-furnace
<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-furnace-3.png" width="400">
      <br /> 
      <b>Test 3</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-furnace-4.png" width="400">
      <br />
      <b>Test 4</b><br>
    </td>
  </tr>
</table>

#### Part 3: Path tracer with next event estimation [Completed]

Rendering result:

1. Cornell Box 

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/cbox/cbox_ems.png" width="400">
      <br /> 
      <b>Cornell Box, 800 * 600, spp: 512</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/cbox/cbox_ems_ref.png" width="400">
      <br />
      <b>Cornell Box, reference</b><br>
    </td>
  </tr>
</table>

2. Veach Scene

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/veach_mi/veach_ems.png" width="400">
      <br /> 
      <b>Veach Scene, 768 * 512, spp: 256</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/veach_mi/veach_ems_ref.png" width="400">
      <br />
      <b>Veach Scene, reference</b><br>
    </td>
  </tr>
</table>

3. Table Scene

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/table/table_ems.png" width="400">
      <br /> 
      <b>Table Scene, 800 * 600, spp: 128</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/table/table_ems_ref.png" width="400">
      <br />
      <b>Table Scene, reference</b><br>
    </td>
  </tr>
</table>

Test result:

1. test-direct
<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-1.png" width="400">
      <br /> 
      <b>Test 1</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-2.png" width="400">
      <br />
      <b>Test 2</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-3.png" width="400">
      <br />
      <b>Test 3</b><br>
    </td>
  </tr>
</table>

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-4.png" width="400">
      <br /> 
      <b>Test 4</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-5.png" width="400">
      <br />
      <b>Test 5</b><br>
    </td>
  </tr>
</table>

2. test-furnace
<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-furnace-1.png" width="400">
      <br /> 
      <b>Test 1</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-furnace-2.png" width="400">
      <br />
      <b>Test 2</b><br>
    </td>
  </tr>
</table>

#### Part 4: Path tracer with Multiple Importance Sampling [Completed]

Rendering result:

1. Cornell Box 
<!-- <div align="center">
  <img src="scenes/pa5/cbox/cbox_mis.png" width="700">
  <p>Cornell Box, 800 * 600, spp: 256</p>
</div> -->

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/cbox/cbox_mis.png" width="400">
      <br /> 
      <b>Cornell Box, 800 * 600, spp: 256</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/cbox/cbox_mis_ref.png" width="400">
      <br />
      <b>Cornell Box, reference</b><br>
    </td>
  </tr>
</table>

2. Veach Scene
<!-- <div align="center">
  <img src="scenes/pa5/veach_mi/veach_mis.png" width="700">
  <p>Veach Scene, 768 * 512, spp: 128</p>
</div> -->

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/veach_mi/veach_mis.png" width="400">
      <br /> 
      <b>Veach Scene, 768 * 512, spp: 128</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/veach_mi/veach_mis_ref.png" width="400">
      <br />
      <b>Veach Scene, reference</b><br>
    </td>
  </tr>
</table>

3. Table Scene
<!-- <div align="center">
  <img src="scenes/pa5/table/table_mis.png" width="700">
  <p>Table Scene, 800 * 600, spp: 64</p>
</div> -->

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/table/table_mis.png" width="400">
      <br /> 
      <b>Table Scene, 800 * 600, spp: 64</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/table/table_mis_ref.png" width="400">
      <br />
      <b>Table Scene, reference</b><br>
    </td>
  </tr>
</table>

Test result:

1. test-direct
<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-11.png" width="400">
      <br /> 
      <b>Test 11</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-12.png" width="400">
      <br />
      <b>Test 12</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-13.png" width="400">
      <br />
      <b>Test 13</b><br>
    </td>
  </tr>
</table>

<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-14.png" width="400">
      <br /> 
      <b>Test 14</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-direct-15.png" width="400">
      <br />
      <b>Test 15</b><br>
    </td>
  </tr>
</table>

2. test-furnace
<table>
  <tr>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-furnace-5.png" width="400">
      <br /> 
      <b>Test 5</b><br>
    </td>
    <td align="center">
      <img src="scenes/pa5/tests/result/test-furnace-6.png" width="400">
      <br />
      <b>Test 6</b><br>
    </td>
  </tr>
</table>

#### Artist Points: Interesting scene

#### Hacker Points: Refraction through rough dielectrics